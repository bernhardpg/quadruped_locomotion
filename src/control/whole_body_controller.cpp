#include "control/whole_body_controller.hpp"

namespace control {
	WholeBodyController::WholeBodyController(int frequency)
		: loop_rate_(frequency)
	{
		ROS_INFO("Running controller at %d Hz", frequency);
		model_name_ = "anymal"; // TODO: Replace with node argument
		
		SetVariablesToZero();

		// TODO: Create function for handling integrators
		q_j_dot_cmd_integrator_.SetSize(kNumJoints);
		q_j_ddot_cmd_integrator_.SetSize(kNumJoints);

		SetupRosTopics();
		SetupRosServices();
		WaitForPublishedTime();
		SpinRosThreads();
		WaitForPublishedState();

		SetRobotMode(kIdle); 
	}

	WholeBodyController::~WholeBodyController()
	{
			ros_node_.shutdown();

			ros_process_queue_.clear();
			ros_process_queue_.disable();
			ros_process_queue_thread_.join();

			ros_publish_queue_.clear();
			ros_publish_queue_.disable();
			ros_publish_queue_thread_.join();
	}

	// **************** //
	// STANDUP SEQUENCE //
	// **************** //

	void WholeBodyController::SetJointInitialConfigTraj()
	{
		const std::vector<double> breaks = {
			0.0, seconds_to_initial_config_ 
		};

		std::vector<Eigen::MatrixXd> samples;
		samples.push_back(q_j_);
		samples.push_back(initial_joint_config);

		auto initial_config_pos_traj
			= CreateFirstOrderHoldTraj(breaks, samples);
		auto initial_config_vel_traj
			= initial_config_pos_traj.derivative(1);

		q_j_cmd_traj_ = initial_config_pos_traj;
		q_j_dot_cmd_traj_ = initial_config_vel_traj;
		traj_end_time_s_ = seconds_to_initial_config_;
	}

	void WholeBodyController::SetComStandupTraj()
	{
		J_task_.resize(1,kNumGenVels);
		J_task_.setZero();
		const int z_index_in_vels = 5;
		J_task_(0, z_index_in_vels) = 1;

		const std::vector<double> breaks =
		{
			0.0, seconds_to_standup_config_
		};

		Eigen::MatrixXd curr_pose(1,1);
		const int z_index_in_coords = 6;
		double curr_height = q_(z_index_in_coords);
		curr_pose << curr_height;

		Eigen::MatrixXd target_pose(1,1);
		target_pose << standing_height_;

		std::vector<Eigen::MatrixXd> samples =
		{
			curr_pose, target_pose	
		};

		auto com_pos_standup_traj
			= CreateFirstOrderHoldTraj(breaks, samples);
		auto com_vel_standup_traj
			= com_pos_standup_traj.derivative(1);

		task_vel_traj_ = com_vel_standup_traj;
		traj_end_time_s_ = seconds_to_standup_config_;
	}

	// TODO: outdated
	void WholeBodyController::SetDanceTraj()
	{
		J_task_.resize(3,kNumGenVels);
		J_task_.setZero();
		int z_index = 2;
		int roll_index = 3;
		int pitch_index = 4;
		J_task_(0, z_index) = 1;
		J_task_(1, roll_index) = 1;
		J_task_(2, pitch_index) = 1;

		const std::vector<double> breaks =
		{
			0.0, 1.0, 2.0, 3.0, 4.0
		};

		Eigen::MatrixXd curr_pose(3,1);
		double curr_height = q_(z_index);
		curr_pose << curr_height, 0, 0;

		double dancing_height = 0.1;

		Eigen::MatrixXd left_roll(3,1);
		left_roll <<
			standing_height_ - dancing_height, -3.14 / 8, -3.14 / 8;

		Eigen::MatrixXd right_roll(3,1);
		right_roll <<
			standing_height_ - dancing_height, 3.14 / 8, 3.14 / 8;

		std::vector<Eigen::MatrixXd> samples =
		{
			curr_pose, left_roll, curr_pose, right_roll,
			curr_pose
		};

		auto com_dance_traj 
			= CreateFirstOrderHoldTraj(breaks, samples);
		auto com_vel_dance_traj 
			= com_dance_traj.derivative(1);

		task_vel_traj_ = com_vel_dance_traj;
		traj_end_time_s_ = 4.0;
	}

	// ********** //
	// CONTROLLER //
	// ********** //

	void WholeBodyController::UpdateJointCommand()
	{
		seconds_in_mode_ = GetElapsedTimeSince(mode_start_time_);
		if (print_frequency_)
		{
			double dt	= GetElapsedTimeSince(last_update_);
			ROS_INFO("Current frequency = %f", 1/dt);
			last_update_ = ros::Time::now();
		}

		if (!controller_ready_) return;

		switch(control_mode_)	
		{
			case kJointTracking:
				DirectJointControl();
				break;
//			case kFeetTracking: TODO: clean up
//				FeetPosControl();
//				break;
			case kSupportConsistentTracking:
				SupportConsistentControl();
				break;
			case kHoQpController:
				{
					// TODO: Add SetComCmd here
					ho_qp_controller_.SetLegCmd(
							r_c_cmd_, r_c_dot_cmd_, r_c_ddot_cmd_,
							legs_in_contact_
							);
					ho_qp_controller_.Update(q_,u_);
					q_j_ddot_cmd_ = ho_qp_controller_.GetJointAccelerationCmd();
					IntegrateJointAccelerations();
					tau_j_cmd_ = ho_qp_controller_.GetJointTorqueCmd();
				}
				break;
			default:
				break;
		}
	}

	void WholeBodyController::IntegrateJointAccelerations()
	{
		q_j_ddot_cmd_integrator_.Integrate(q_j_ddot_cmd_);
		q_j_dot_cmd_ = q_j_ddot_cmd_integrator_.GetIntegral();
		q_j_dot_cmd_integrator_.Integrate(q_j_dot_cmd_);
		q_j_cmd_ = q_j_dot_cmd_integrator_.GetIntegral();
	}

	void WholeBodyController::DirectJointControl()
	{
		q_j_cmd_ = EvalPosTrajAtTime(
				q_j_cmd_traj_, seconds_in_mode_
				);
		q_j_dot_cmd_ = EvalVelTrajAtTime(
				q_j_dot_cmd_traj_, seconds_in_mode_
				);
	}

	// TODO: Move this to its own class
	void WholeBodyController::SupportConsistentControl()
	{
		robot_dynamics_.SetState(q_,u_);
		const std::vector<int> all_legs = {0,1,2,3};
		Eigen::MatrixXd J_constraint = robot_dynamics_
			.GetStackedContactJacobianInW(all_legs);
		Eigen::MatrixXd N_constraint =
			CalcSquareNullSpaceProjMatrix(J_constraint);

		Eigen::MatrixXd w_task = EvalVelTrajAtTime(
				task_vel_traj_, seconds_in_mode_
				);

		Eigen::MatrixXd u_0_cmd
			= CalcPseudoInverse(J_task_ * N_constraint) * w_task;
		Eigen::MatrixXd u_cmd = N_constraint * u_0_cmd;

		q_j_dot_cmd_ = u_cmd.block<kNumJoints,1>(kNumTwistCoords,0);

		q_j_dot_cmd_integrator_.Integrate(q_j_dot_cmd_);
		q_j_cmd_ = q_j_dot_cmd_integrator_.GetIntegral();
	}

	// ************* //
	// STATE MACHINE // 
	// ************* //

	void WholeBodyController::SetRobotMode(RobotMode target_mode)
	{
		controller_ready_ = false;
		switch(target_mode)
		{
			case kIdle:
				{
					ROS_INFO("Setting mode to IDLE");
					SetJointInitialConfigTraj();
					control_mode_ = kJointTracking; 
				}
				break;
			case kStandup:
				{
					ROS_INFO("Setting mode to STANDUP");
					q_j_dot_cmd_integrator_.Reset();
					q_j_dot_cmd_integrator_.SetIntegral(q_j_);
					SetComStandupTraj();
					control_mode_ = kSupportConsistentTracking;
				}
				break;
			case kDance:
				{
					ROS_INFO("Setting mode to DANCE");
					q_j_dot_cmd_integrator_.Reset();
					q_j_dot_cmd_integrator_.SetIntegral(q_j_);
					SetDanceTraj();
					control_mode_ = kSupportConsistentTracking;
				}
				break;
			case kWalk:
				{
					ROS_INFO("Setting mode to WALK");
					q_j_dot_cmd_integrator_.Reset();
					// Start joint position at current position
					q_j_dot_cmd_integrator_.SetIntegral(q_j_);
					q_j_ddot_cmd_integrator_.Reset();
					control_mode_ = kHoQpController;
				}
				break;
			default:
				break;
		}
		SetModeStartTime();
		controller_ready_ = true;
	}

	// *** //
	// ROS //
	// *** //

	void WholeBodyController::SetupRosTopics()
	{
		SetupJointCmdAdvertisement();
		SetupStateSubscriptions();
		SetupLegCmdSubscriptions();
	}

	void WholeBodyController::SetupJointCmdAdvertisement()
	{
		ros::AdvertiseOptions q_j_cmd_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/q_j_cmd",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		ros::AdvertiseOptions q_j_dot_cmd_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/q_j_dot_cmd",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		ros::AdvertiseOptions tau_j_cmd_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/tau_j_cmd",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		q_j_cmd_pub_= ros_node_.advertise(q_j_cmd_ao);
		q_j_dot_cmd_pub_= ros_node_.advertise(q_j_dot_cmd_ao);
		tau_j_cmd_pub_ = ros_node_.advertise(tau_j_cmd_ao);
	}

	void WholeBodyController::SetupStateSubscriptions()
	{
		ros::SubscribeOptions gen_coord_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + model_name_ + "/gen_coord",
					1,
					boost::bind(&WholeBodyController::OnGenCoordMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		ros::SubscribeOptions gen_vel_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + model_name_ + "/gen_vel",
					1,
					boost::bind(&WholeBodyController::OnGenVelMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		gen_coord_sub_ = ros_node_.subscribe(gen_coord_so);
		gen_vel_sub_ = ros_node_.subscribe(gen_vel_so);
	}

	void WholeBodyController::SetupLegCmdSubscriptions()
	{
		ros::SubscribeOptions legs_pos_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/legs_pos_cmd",
					1,
					boost::bind(&WholeBodyController::OnLegsPosCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		ros::SubscribeOptions legs_vel_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/legs_vel_cmd",
					1,
					boost::bind(&WholeBodyController::OnLegsVelCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		ros::SubscribeOptions legs_acc_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/legs_acc_cmd",
					1,
					boost::bind(&WholeBodyController::OnLegsAccCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		ros::SubscribeOptions legs_contact_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/legs_contact_cmd",
					1,
					boost::bind(&WholeBodyController::OnLegsContactCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		legs_pos_cmd_sub_ = ros_node_.subscribe(legs_pos_cmd_so);
		legs_vel_cmd_sub_ = ros_node_.subscribe(legs_vel_cmd_so);
		legs_acc_cmd_sub_ = ros_node_.subscribe(legs_acc_cmd_so);
		legs_contact_cmd_sub_ = ros_node_.subscribe(legs_contact_cmd_so);
	}

	void WholeBodyController::SetupRosServices()
	{
		ros::AdvertiseServiceOptions cmd_standup_aso =
			ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
            "/" + model_name_ + "/standup",
            boost::bind(&WholeBodyController::CmdStandupService, this, _1, _2),
            ros::VoidPtr(),
            &this->ros_process_queue_
        );

		cmd_standup_service_ = ros_node_.advertiseService(cmd_standup_aso);

		ros::AdvertiseServiceOptions cmd_dance_aso =
			ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
            "/" + model_name_ + "/dance",
            boost::bind(&WholeBodyController::CmdDanceService, this, _1, _2),
            ros::VoidPtr(),
            &this->ros_process_queue_
        );

		cmd_dance_service_ = ros_node_.advertiseService(cmd_dance_aso);

		ros::AdvertiseServiceOptions cmd_walk_aso =
			ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
            "/" + model_name_ + "/walk",
            boost::bind(&WholeBodyController::CmdWalkService, this, _1, _2),
            ros::VoidPtr(),
            &this->ros_process_queue_
        );

		cmd_walk_service_ = ros_node_.advertiseService(cmd_walk_aso);
	}

	bool WholeBodyController::CmdStandupService(
					const std_srvs::Empty::Request &_req,
					std_srvs::Empty::Response &_res
			)
	{
		SetRobotMode(kStandup);
		return true;
	}

	bool WholeBodyController::CmdDanceService(
					const std_srvs::Empty::Request &_req,
					std_srvs::Empty::Response &_res
			)
	{
		SetRobotMode(kDance);
		return true;
	}

	bool WholeBodyController::CmdWalkService(
					const std_srvs::Empty::Request &_req,
					std_srvs::Empty::Response &_res
			)
	{
		SetRobotMode(kWalk);
		return true;
	}

	void WholeBodyController::SpinRosThreads()
	{
		ros_process_queue_thread_ = std::thread(
				std::bind(&WholeBodyController::ProcessQueueThread, this)
				);
		ros_publish_queue_thread_ = std::thread(
				std::bind(&WholeBodyController::PublishQueueThread, this)
				);
	}

	void WholeBodyController::ProcessQueueThread()
	{
		static const double timeout = 0.01; // TODO: Verify that this is okay
		while (ros_node_.ok())
		{
			ros_process_queue_.callAvailable(
					ros::WallDuration(timeout)
					);
		}
	}

	void WholeBodyController::PublishQueueThread()
	{
		while (ros_node_.ok())
		{
			UpdateJointCommand();
			PublishJointPosCmd();
			PublishJointVelCmd();
			PublishJointTorqueCmd();
			loop_rate_.sleep();	
		}
	}

	void WholeBodyController::PublishJointPosCmd()
	{
		std_msgs::Float64MultiArray q_j_cmd_msg;
		tf::matrixEigenToMsg(q_j_cmd_, q_j_cmd_msg);
		q_j_cmd_pub_.publish(q_j_cmd_msg);
	}
	
	void WholeBodyController::PublishJointVelCmd()
	{
		std_msgs::Float64MultiArray q_j_dot_cmd_msg;
		tf::matrixEigenToMsg(q_j_dot_cmd_, q_j_dot_cmd_msg);
		q_j_dot_cmd_pub_.publish(q_j_dot_cmd_msg);
	}

	void WholeBodyController::PublishJointTorqueCmd()
	{
		std_msgs::Float64MultiArray tau_j_cmd_msg;
		tf::matrixEigenToMsg(tau_j_cmd_, tau_j_cmd_msg);
		tau_j_cmd_pub_.publish(tau_j_cmd_msg);
	}

	void WholeBodyController::OnGenCoordMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		received_first_state_msg_ = true;
		SetGenCoords(msg->data);
	}

	void WholeBodyController::OnGenVelMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		received_first_state_msg_ = true;
		SetGenVels(msg->data);
	}

	void WholeBodyController::OnLegsPosCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetLegPosCmd(msg->data);
	}

	void WholeBodyController::OnLegsVelCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetLegVelCmd(msg->data);
	}

	void WholeBodyController::OnLegsAccCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetLegAccCmd(msg->data);
	}

	void WholeBodyController::OnLegsContactCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetLegContactCmd(msg->data);
	}


	// ***************** //
	// SETTERS & GETTERS //
	// ***************** //

	joint_vector_t WholeBodyController::GetJointsPos()
	{
		joint_vector_t q_j = 
			q_.block<kNumJoints,1>(kNumPoseCoords,0);
		return q_j;
	}

	joint_vector_t WholeBodyController::GetJointsVel()
	{
		joint_vector_t q_j_dot = 
			u_.block<kNumJoints,1>(kNumTwistCoords,0);
		return q_j_dot;
	}

	void WholeBodyController::SetGenCoords(const std::vector<double> &gen_coords)
	{
		for (int i = 0; i < kNumGenCoords; ++i)
			q_(i) = gen_coords[i];

		q_j_ = GetJointsPos();
	}

	void WholeBodyController::SetGenVels(const std::vector<double> &gen_vels)
	{
		for (int i = 0; i < 18; ++i)
			u_(i) = gen_vels[i];

		q_j_dot_ = GetJointsVel();
	}

	void WholeBodyController::SetLegPosCmd(
			const std::vector<double> &leg_pos_cmd
			)
	{
		for (int i = 0; i < leg_pos_cmd.size(); ++i)
			r_c_cmd_(i) = leg_pos_cmd[i];
	}

	void WholeBodyController::SetLegVelCmd(
			const std::vector<double> &leg_vel_cmd
			)
	{
		for (int i = 0; i < leg_vel_cmd.size(); ++i)
			r_c_dot_cmd_(i) = leg_vel_cmd[i];
	}

	void WholeBodyController::SetLegAccCmd(
			const std::vector<double> &leg_acc_cmd
			)
	{
		for (int i = 0; i < leg_acc_cmd.size(); ++i)
			r_c_ddot_cmd_(i) = leg_acc_cmd[i];
	}

	void WholeBodyController::SetLegContactCmd(
			const std::vector<double> &leg_contact_cmd
			)
	{
		// TODO: this conversion should happen elsewhere
		legs_in_contact_.clear();
		for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
		{
			if (leg_contact_cmd[leg_i] == 1)
			{
				legs_in_contact_.push_back(leg_i);
			}
		}
	}

	// **************** //
	// HELPER FUNCTIONS //
	// **************** //
	
	Eigen::MatrixXd WholeBodyController::EvalPosTrajAtTime(
			drake::trajectories::PiecewisePolynomial<double> traj,
			double curr_time)
	{
		if (curr_time > traj_end_time_s_)
		{
			return traj.value(traj_end_time_s_);
		}
		return traj.value(curr_time);
	}

	Eigen::MatrixXd WholeBodyController::EvalVelTrajAtTime(
			drake::trajectories::PiecewisePolynomial<double> traj,
			double curr_time)
	{
		if (curr_time > traj_end_time_s_)
		{
			return Eigen::MatrixXd::Zero(traj.rows(), traj.cols());
		}
		return traj.value(curr_time);
	}

	void WholeBodyController::WaitForPublishedTime()
	{
		while (ros::Time::now().toSec() == 0.0); 
		ROS_INFO("Received published time");
	}

	void WholeBodyController::WaitForPublishedState()
	{
		while (!received_first_state_msg_);
		ROS_INFO("Received published state");
	}

	drake::trajectories::PiecewisePolynomial<double>
		WholeBodyController::CreateFirstOrderHoldTraj(
			std::vector<double> breaks,
			std::vector<Eigen::MatrixXd> samples
			)
	{
		drake::trajectories::PiecewisePolynomial traj
			= drake::trajectories::PiecewisePolynomial<double>
					::FirstOrderHold(breaks, samples);
		return traj;
	}

	void WholeBodyController::SetVariablesToZero()
	{
		q_.setZero(); // NOTE: sets quaternion to zero too!
		u_.setZero();

		q_j_.resize(kNumJoints, 1);
		q_j_.setZero();
		q_j_dot_.resize(kNumJoints, 1);
		q_j_dot_.setZero();

		q_j_cmd_.setZero();
		q_j_dot_cmd_.setZero();

		q_j_dot_cmd_integrator_ = Integrator(kNumJoints);
		q_j_dot_cmd_integrator_.Reset();

		legs_in_contact_.clear();

		r_c_cmd_.resize(k3D * kNumLegs);
		r_c_cmd_.setZero();

		r_c_dot_cmd_.resize(k3D * kNumLegs);
		r_c_dot_cmd_.setZero();

		r_c_ddot_cmd_.resize(k3D * kNumLegs);
		r_c_ddot_cmd_.setZero();
	}
	
	void WholeBodyController::SetModeStartTime()
	{
		mode_start_time_ = ros::Time::now();
		last_update_ = ros::Time::now();
	}

	double WholeBodyController::GetElapsedTimeSince(ros::Time t)
	{
		double elapsed_time = (ros::Time::now() - t).toSec();
		return elapsed_time; 
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "whole_body_controller",
			ros::init_options::NoSigintHandler
			);
	ROS_INFO("Starting WholeBodyController");
	int frequency = 400;
	control::WholeBodyController whole_body_controller(frequency);
	ros::spin();

	return 0;
}
