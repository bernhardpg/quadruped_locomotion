#include "control/controller.hpp"

// TODO: remove all 'this' references

namespace control {
	Controller::Controller(int frequency)
		: loop_rate_(frequency)
	{
		ROS_INFO("Running controller at %d Hz", frequency);
		model_name_ = "anymal"; // TODO: Replace with node argument
		
		// TODO: Move into function?
		q_.setZero();
		u_.setZero();
		q_j_cmd_.setZero();
		q_j_dot_cmd_.setZero();

		InitRos();
		SetStartTime();
		SpinRosThreads();

		// Wait for state to get published
		while (q_.isZero(0) || u_.isZero(0));

		InitController();
		CreateStandupTrajectory(); 
		controller_initialized_ = true; // TODO: cleanup

		RunStandupSequence();
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

		q_j_cmd_.setZero();
		q_j_dot_cmd_.setZero();

		q_j_dot_cmd_integrator_ = Integrator(kNumJoints_);
		q_j_dot_cmd_integrator_.Reset();
	}

	// **************** //
	// STANDUP SEQUENCE //
	// **************** //

	void Controller::RunStandupSequence()
	{
		ROS_INFO("Starting standup sequence");
	}

	void Controller::CreateStandupTrajectory()
	{
		// Three standup steps 
		const std::vector<double> breaks = { 0.0, 10.0};
		std::vector<Eigen::MatrixXd> samples;

		Eigen::MatrixXd start_conf(12,1);
		start_conf = robot_dynamics_.GetFeetPositions(q_);

		Eigen::MatrixXd neutral_conf(12,1);
		Eigen::Matrix<double, 19, 1> q_neutral;
		q_neutral.block<7,1>(0,0) = q_.block<7,1>(0,0);
		q_neutral.block<12,1>(7,0) = Eigen::VectorXd::Zero(12);
		neutral_conf = robot_dynamics_.GetFeetPositions(q_neutral);

		Eigen::MatrixXd apex_conf = neutral_conf;
		for (int leg_i = 0; leg_i < n_legs_; ++leg_i)
			apex_conf(2 + leg_i * n_dims_) = swing_height_;

		Eigen::MatrixXd touchdown_conf = neutral_conf;
		for (int leg_i = 0; leg_i < n_legs_; ++leg_i)
			touchdown_conf(2 + leg_i * n_dims_) = 0.0;

		samples.push_back(start_conf);
		samples.push_back(apex_conf);
		//samples.push_back(touchdown_conf); // TODO: Only lift legs for now

		// Use drake piecewise polynomials for easy trajectory construction
		standup_pos_traj_ =
			drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
					breaks, samples
					);
		standup_vel_traj_ = standup_pos_traj_.derivative(1);
	}

	void Controller::CalcJointCmd()
	{
		t_ = GetElapsedTimeSince(start_time_);
		CalcFeetTrackingError();

		J_feet_pos_ = robot_dynamics_
			.GetStackedFeetJacobianPos(q_)
			.block(0,6,12,12); // Get rid of jacobian for body
		Eigen::MatrixXd J_inv = CalcPseudoInverse(J_feet_pos_);
		
		// Position controller
		q_j_dot_cmd_ = J_inv
			* (k_pos_p_ * feet_pos_error_ + feet_vel_ff_);

		q_j_dot_cmd_integrator_.Integrate(q_j_dot_cmd_);
		q_j_cmd_ = q_j_dot_cmd_integrator_.GetIntegral();
	}

	void Controller::CalcFeetTrackingError()
	{
		feet_pos_error_ =
			standup_pos_traj_.value(t_)
			- robot_dynamics_.GetFeetPositions(q_);

		feet_vel_ff_ = standup_vel_traj_.value(t_);
	}

	// *** //
	// ROS //
	// *** //

	void Controller::InitRos()
	{
		ROS_INFO("Initialized controller node");

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
			if (!controller_initialized_)
				continue;

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

	void Controller::SetGenCoords(const std::vector<double> &gen_coords)
	{
		for (int i = 0; i < kNumGenCoords_; ++i)
			q_(i) = gen_coords[i];
	}

	void Controller::SetGenVels(const std::vector<double> &gen_vels)
	{
		for (int i = 0; i < 18; ++i)
			u_(i) = gen_vels[i];
	}

	// **************** //
	// HELPER FUNCTIONS //
	// **************** //

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
		// Moore-Penrose right inverse: A^t (A A^t)
		Eigen:: MatrixXd pseudo_inverse =
			A.transpose() * (A * A.transpose()).inverse();
		return pseudo_inverse;
	}
}
