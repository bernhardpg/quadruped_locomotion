#include "control/whole_body_controller.hpp"

namespace control {
	WholeBodyController::WholeBodyController(int frequency)
		: loop_rate_(frequency)
	{
		ROS_INFO("Running controller at %d Hz", frequency);
		
		SetVariablesToZero();

		SetControlMode(kJointTracking);

		SetupRosTopics();
		WaitForPublishedTime();
		SpinRosThreads();
		WaitForPublishedState();

		InitIntegrators();
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

	// ************* //
	// INIT SEQUENCE //
	// ************* //

	void WholeBodyController::CreateInitialJointConfigTraj()
	{
		const std::vector<double> breaks = {
			0.0, init_sequence_duration_ 
		};

		std::vector<Eigen::MatrixXd> samples;
		samples.push_back(q_j_);
		samples.push_back(kInitialJointConfig);

		auto initial_config_pos_traj =
			drake::trajectories::PiecewisePolynomial<double>
					::FirstOrderHold(breaks, samples);
		auto initial_config_vel_traj
			= initial_config_pos_traj.derivative(1);

		q_j_init_traj_ = initial_config_pos_traj;
		q_j_dot_init_traj_ = initial_config_vel_traj;
	}

	// ********** //
	// CONTROLLER //
	// ********** //

	void WholeBodyController::SetControlMode(ControlMode target_mode)
	{
		switch(target_mode)
		{
			case kJointTracking:
				init_start_time_ = ros::Time::now();
				CreateInitialJointConfigTraj();
				ROS_INFO("Switching to direct joint tracking");
				break;
			case kHoQpController:
				ROS_INFO("Switching to Hierarchical Optimization Controller");
				SetIntegratorsToCurrentState();
				break;
		}

		control_mode_ = target_mode;
		controller_ready_ = true;
	}

	void WholeBodyController::UpdateJointCommand()
	{
		if (print_frequency_)
		{
			double dt	= GetElapsedTimeSince(last_update_);
			ROS_INFO("Current frequency = %f", 1/dt);
			last_update_ = ros::Time::now();
		}

		switch(control_mode_)	
		{
			case kJointTracking:
				DirectJointControl();
				break;
			case kHoQpController:
				{
					ho_qp_controller_.SetBaseCmd(
							r_cmd_, r_dot_cmd_, r_ddot_cmd_
							);
					ho_qp_controller_.SetLegCmd(
							r_c_cmd_, r_c_dot_cmd_, r_c_ddot_cmd_,
							contact_pattern_cmd_
							);
					ho_qp_controller_.CalcJointCmd(q_,u_);
					q_j_ddot_cmd_ = ho_qp_controller_.GetJointAccelerationCmd();
					SetJointCmdFromIntegrators();
					tau_j_cmd_ = ho_qp_controller_.GetJointTorqueCmd();
				}
				break;
		}
	}

	void WholeBodyController::DirectJointControl()
	{
		double time = GetElapsedTimeSince(init_start_time_);
		if (time > init_sequence_duration_)
		{
			q_j_cmd_ = q_j_init_traj_.value(init_sequence_duration_);
			q_j_dot_cmd_ = Eigen::VectorXd::Zero(kNumJoints);
		}
		else
		{
			q_j_cmd_ = q_j_init_traj_.value(time);
			q_j_dot_cmd_ = q_j_dot_init_traj_.value(time);
		}
	}

	// *********** //
	// INTEGRATORS //
	// *********** //

	void WholeBodyController::InitIntegrators()
	{
		q_j_dot_cmd_integrator_.SetSize(kNumJoints);
		q_j_dot_cmd_integrator_.Reset();

		q_j_ddot_cmd_integrator_.SetSize(kNumJoints);
		q_j_ddot_cmd_integrator_.Reset();
	}

	void WholeBodyController::SetIntegratorsToCurrentState()
	{
		InitIntegrators();
		q_j_dot_cmd_integrator_.SetIntegral(q_j_);
		q_j_ddot_cmd_integrator_.SetIntegral(q_j_dot_);
	}

	void WholeBodyController::SetJointCmdFromIntegrators()
	{
		q_j_ddot_cmd_integrator_.Integrate(q_j_ddot_cmd_);
		q_j_dot_cmd_ = q_j_ddot_cmd_integrator_.GetIntegral();

		q_j_dot_cmd_integrator_.Integrate(q_j_dot_cmd_);
		q_j_cmd_ = q_j_dot_cmd_integrator_.GetIntegral();
	}

	// *** //
	// ROS //
	// *** //

	void WholeBodyController::SetupRosTopics()
	{
		SetupJointCmdAdvertisement();
		SetupStateSubscriptions();
		SetupBaseCmdSubscriptions();
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

	void WholeBodyController::SetupBaseCmdSubscriptions()
	{
		ros::SubscribeOptions base_pos_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/base_pos_cmd",
					1,
					boost::bind(&WholeBodyController::OnBasePosCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		ros::SubscribeOptions base_vel_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/base_vel_cmd",
					1,
					boost::bind(&WholeBodyController::OnBaseVelCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		ros::SubscribeOptions base_acc_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/base_acc_cmd",
					1,
					boost::bind(&WholeBodyController::OnBaseAccCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		base_pos_cmd_sub_ = ros_node_.subscribe(base_pos_cmd_so);
		base_vel_cmd_sub_ = ros_node_.subscribe(base_vel_cmd_so);
		base_acc_cmd_sub_ = ros_node_.subscribe(base_acc_cmd_so);
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
			if (!controller_ready_)
				continue;

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

	void WholeBodyController::OnBasePosCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetBasePosCmd(msg->data);
	}

	void WholeBodyController::OnBaseVelCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetBaseVelCmd(msg->data);
	}

	void WholeBodyController::OnBaseAccCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetBaseAccCmd(msg->data);
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

	void WholeBodyController::SetBasePosCmd(
			const std::vector<double> &base_pos_cmd
			)
	{
		if (control_mode_ != kHoQpController)
			SetControlMode(kHoQpController);
		
		for (int i = 0; i < base_pos_cmd.size(); ++i)
			r_cmd_(i) = base_pos_cmd[i];
	}

	void WholeBodyController::SetBaseVelCmd(
			const std::vector<double> &base_vel_cmd
			)
	{
		for (int i = 0; i < base_vel_cmd.size(); ++i)
			r_dot_cmd_(i) = base_vel_cmd[i];
	}

	void WholeBodyController::SetBaseAccCmd(
			const std::vector<double> &base_acc_cmd
			)
	{
		for (int i = 0; i < base_acc_cmd.size(); ++i)
			r_ddot_cmd_(i) = base_acc_cmd[i];
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
		for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
			contact_pattern_cmd_(leg_i) = leg_contact_cmd[leg_i];
	}

	// **************** //
	// HELPER FUNCTIONS //
	// **************** //
	
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

	void WholeBodyController::SetVariablesToZero()
	{
		q_.setZero(); // NOTE: sets quaternion to zero too!
		u_.setZero();

		q_j_.resize(kNumJoints);
		q_j_.setZero();
		q_j_dot_.resize(kNumJoints);
		q_j_dot_.setZero();

		q_j_cmd_.resize(kNumJoints);
		q_j_cmd_.setZero();

		q_j_dot_cmd_.resize(kNumJoints);
		q_j_dot_cmd_.setZero();

		q_j_ddot_cmd_.resize(kNumJoints);
		q_j_ddot_cmd_.setZero();

		tau_j_cmd_.resize(kNumJoints);
		tau_j_cmd_.setZero();

		SetZeroBaseCmdMotion();
		SetZeroLegCmdMotion();
	}

	void WholeBodyController::SetZeroBaseCmdMotion()
	{
		r_cmd_.resize(k3D);
		r_cmd_.setZero();

		r_dot_cmd_.resize(k3D);
		r_dot_cmd_.setZero();

		r_ddot_cmd_.resize(k3D);
		r_ddot_cmd_.setZero();
	}

	void WholeBodyController::SetZeroLegCmdMotion()
	{
		// all legs in contact by default
		contact_pattern_cmd_ = Eigen::VectorXd::Ones(kNumLegs);

		r_c_cmd_.resize(k3D * kNumLegs);
		r_c_cmd_.setZero();

		r_c_dot_cmd_.resize(k3D * kNumLegs);
		r_c_dot_cmd_.setZero();

		r_c_ddot_cmd_.resize(k3D * kNumLegs);
		r_c_ddot_cmd_.setZero();
	}

	double WholeBodyController::GetElapsedTimeSince(ros::Time t)
	{
		double elapsed_time = (ros::Time::now() - t).toSec();
		return elapsed_time; 
	}
}

// ******** //
// ROS NODE //
// ******** //

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
