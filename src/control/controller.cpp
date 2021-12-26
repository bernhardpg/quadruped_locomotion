#include "control/controller.hpp"

// TODO: remove all 'this' references

namespace control {
	Controller::Controller(int frequency)
		: loop_rate_(frequency)
	{
		ROS_INFO("Running controller at %d Hz", frequency);
		model_name_ = "anymal"; // TODO: Replace with node argument
		
		SetStateVariablesToZero();

		InitRos();
		SetStartTime();
		SpinRosThreads();

		// Wait for state to get published
		//while (q_.isZero(0) || u_.isZero(0))

		InitController();
		CreateInitialConfigTraj();
		controller_ready_ = true; // TODO: cleanup
	}

	Controller::~Controller()
	{
			ros_node_.shutdown();

			ros_process_queue_.clear();
			ros_process_queue_.disable();
			ros_process_queue_thread_.join();

			ros_publish_queue_.clear();
			ros_publish_queue_.disable();
			ros_publish_queue_thread_.join();
	}

	void Controller::InitController()
	{
		feet_pos_error_.setZero();
		feet_vel_ff_.setZero();

		// TODO: Move this? This is duplicated right now
		q_j_cmd_.setZero();
		q_j_dot_cmd_.setZero();

		q_j_dot_cmd_integrator_ = Integrator(kNumJoints);
		q_j_dot_cmd_integrator_.Reset();
	}

	// **************** //
	// STANDUP SEQUENCE //
	// **************** //

	void Controller::CreateInitialConfigTraj()
	{

		const std::vector<double> breaks = {
			0.0, seconds_to_initial_config_
		};

		std::vector<Eigen::MatrixXd> samples;
		samples.push_back(q_j_);
		samples.push_back(initial_joint_config);

		q_j_initial_config_traj_ = CreateFirstOrderHoldTraj(breaks, samples);
		q_j_dot_initial_config_traj_ = q_j_initial_config_traj_.derivative(1);
	}

	void Controller::CreateStandupTraj()
	{
		// TODO: Move this to some state handler
		q_j_dot_cmd_integrator_.SetIntegral(q_j_); // TODO: This should be set when initial config is reached

		const std::vector<double> breaks = {
			0.0, seconds_to_standup_config_
		};

		Eigen::MatrixXd feet_start_pos(kNumLegs * kNumPosDims,1);
		feet_start_pos = robot_dynamics_.GetFeetPositions(q_);

		Eigen::MatrixXd feet_standing_pos = feet_start_pos;
		for (int foot_i = 0; foot_i < kNumLegs; ++foot_i)
		{
			int foot_i_z_index = 2 + foot_i * kNumPosDims;
			feet_standing_pos(foot_i_z_index) = standing_height_;
		}

		std::vector<Eigen::MatrixXd> samples;
		samples.push_back(feet_start_pos);
		samples.push_back(feet_standing_pos);

		feet_standup_pos_traj_ = CreateFirstOrderHoldTraj(breaks, samples);
		feet_standup_vel_traj_ = feet_standup_pos_traj_.derivative(1);
		created_standup_traj_ = true; // TODO: Remove this?
	}

	void Controller::JacobianController()
	{
		t_ = GetElapsedTimeSince(start_time_);

		feet_pos_error_ =
			feet_standup_pos_traj_.value(t_)
			- robot_dynamics_.GetFeetPositions(q_);

		feet_vel_ff_ = feet_standup_vel_traj_.value(t_);

		J_feet_pos_ = robot_dynamics_
			.GetStackedFeetJacobianPos(q_)
			.block(0,6,12,12); // Get rid of jacobian for body
		Eigen::MatrixXd J_inv = CalcPseudoInverse(J_feet_pos_);
		
		// Position controller
		q_j_dot_cmd_ = J_inv
			* (k_pos_p_ * feet_pos_error_ + feet_vel_ff_);

		q_j_dot_cmd_integrator_.Integrate(q_j_dot_cmd_);
		q_j_cmd_ = q_j_dot_cmd_integrator_.GetIntegral();
		std::cout << std::fixed << std::setprecision(2)
			<< feet_pos_error_.transpose() << std::endl 
			<< feet_vel_ff_.transpose() << std::endl
			<< q_j_cmd_.transpose() << std::endl
			<< q_j_dot_cmd_.transpose() << std::endl << std::endl;

		SetStateToCmd();
	}

	// TODO: only for testing
	void Controller::SetStateToCmd()
	{
		q_.block<3,1>(0,0).setZero();
		q_(3) = 1;
		q_.block<3,1>(4,0).setZero();
		q_.block<12,1>(7,0) = q_j_cmd_;

		u_.block<3,1>(0,0).setZero();
		u_.block<3,1>(3,0).setZero();
		u_.block<12,1>(6,0) = q_j_dot_cmd_;
	}

	void Controller::CalcJointCmd()
	{
		t_ = GetElapsedTimeSince(start_time_);
		// TODO: clean up this
		if (t_ < standup_start_time_)
		{
			q_j_cmd_ = q_j_ref_traj_.value(standup_start_time_);
			q_j_dot_cmd_.setZero();
		}
		else if (t_ > standup_end_time_ + 2) // TODO: wait two seconds
		{
			if (!created_standup_traj_)
			{
				CreateStandupTraj(); 
			}
			if (created_standup_traj_)	
			{
				JacobianController();
			}
		} 
		else
		{
			q_j_cmd_ = q_j_ref_traj_.value(t_);
			q_j_dot_cmd_ = q_j_dot_ref_traj_.value(t_);
		}
	}

	// *** //
	// ROS //
	// *** //

	void Controller::InitRos()
	{
		ROS_INFO("Initialized controller node");

		// TODO: All of these constructions can be moved into their own functions, this is essentially just repeated code.

		// Set up advertisements
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

		q_j_cmd_pub_= ros_node_.advertise(q_j_cmd_ao);
		q_j_dot_cmd_pub_= ros_node_.advertise(q_j_dot_cmd_ao);

		// Set up subscriptions
		ros::SubscribeOptions gen_coord_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_name_ + "/gen_coord",
					1,
					boost::bind(&Controller::OnGenCoordMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		ros::SubscribeOptions gen_vel_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_name_ + "/gen_vel",
					1,
					boost::bind(&Controller::OnGenVelMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		gen_coord_sub_ = ros_node_.subscribe(gen_coord_so);
		gen_vel_sub_ = ros_node_.subscribe(gen_vel_so);

		ROS_INFO("Finished setting up ros topics");
	}

	void Controller::SpinRosThreads()
	{
		this->ros_process_queue_thread_ = std::thread(
				std::bind(&Controller::ProcessQueueThread, this)
				);
		this->ros_publish_queue_thread_ = std::thread(
				std::bind(&Controller::PublishQueueThread, this)
				);
	}

	void Controller::ProcessQueueThread()
	{
		static const double timeout = 0.01; // TODO: I should check this number
		while (ros_node_.ok())
		{
			ros_process_queue_.callAvailable(
					ros::WallDuration(timeout)
					);
		}
	}

	void Controller::PublishQueueThread()
	{
		while (ros_node_.ok())
		{
			if (!controller_ready_) continue;

			//CalcJointCmdStandup();
			CalcJointCmd();

			std_msgs::Float64MultiArray q_j_cmd_msg;
			tf::matrixEigenToMsg(q_j_cmd_, q_j_cmd_msg);
			q_j_cmd_pub_.publish(q_j_cmd_msg);

			std_msgs::Float64MultiArray q_j_dot_cmd_msg;
			tf::matrixEigenToMsg(q_j_dot_cmd_, q_j_dot_cmd_msg);
			q_j_dot_cmd_pub_.publish(q_j_dot_cmd_msg);

			loop_rate_.sleep();	
		}
	}

	void Controller::OnGenCoordMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetGenCoords(msg->data);
	}

	void Controller::OnGenVelMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetGenVels(msg->data);
	}


	// ***************** //
	// SETTERS & GETTERS //
	// ***************** //

	joint_vector_t Controller::GetJointsPos()
	{
		joint_vector_t q_j = 
			q_.block<kNumJoints,1>(kJointIndexInGenCoords,0);
		return q_j;
	}

	joint_vector_t Controller::GetJointsVel()
	{
		joint_vector_t q_j_dot = 
			u_.block<kNumJoints,1>(kJointIndexInGenVels,0);
		return q_j_dot;
	}

	void Controller::SetGenCoords(const std::vector<double> &gen_coords)
	{
		for (int i = 0; i < kNumGenCoords; ++i)
			q_(i) = gen_coords[i];

		q_j_ = GetJointsPos();
	}

	void Controller::SetGenVels(const std::vector<double> &gen_vels)
	{
		for (int i = 0; i < 18; ++i)
			u_(i) = gen_vels[i];

		q_j_dot_ = GetJointsPos();
	}

	// **************** //
	// HELPER FUNCTIONS //
	// **************** //

	drake::trajectories::PiecewisePolynomial<double>
		Controller::CreateFirstOrderHoldTraj(
			std::vector<double> breaks,
			std::vector<Eigen::VectorXd> samples
			)
	{
		drake::trajectories::PiecewisePolynomial traj
			= drake::trajectories::PiecewisePolynomial<double>
					::FirstOrderHold(breaks, samples);
		return traj;
	}

	void Controller::SetStateVariablesToZero()
	{
		q_.setZero();
		Eigen::Matrix<double,4,1> identity_quaternion;
		identity_quaternion << 1, 0, 0, 0;
		q_.block<4,1>(3,0) = identity_quaternion;

		u_.setZero();

		q_j_.resize(kNumJoints, 1);
		q_j_.setZero();
		q_j_dot_.resize(kNumJoints, 1);
		q_j_dot_.setZero();

		q_j_cmd_.setZero();
		q_j_dot_cmd_.setZero();
	}
	
	void Controller::SetStartTime()
	{
		// Wait for /clock to get published
		while (ros::Time::now().toSec() == 0.0); 
		start_time_ = ros::Time::now();
		ROS_INFO("Start time: %f", start_time_.toSec());
	}

	double Controller::GetElapsedTimeSince(ros::Time t)
	{
		double elapsed_time = (ros::Time::now() - t).toSec();
		return elapsed_time; 
	}

	Eigen::MatrixXd Controller::CalcPseudoInverse(Eigen::MatrixXd A)
	{
		Eigen::MatrixXd eye =
			Eigen::MatrixXd::Identity(A.rows(), A.rows()); // same size as A*A^t
		double damping = 1;

		// Moore-Penrose right inverse: A^t (A A^t)
		Eigen:: MatrixXd pseudo_inverse =
			A.transpose() * (A * A.transpose() + damping * eye).inverse();
		return pseudo_inverse;
	}
}
