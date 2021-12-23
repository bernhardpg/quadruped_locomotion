#include "control/controller.hpp"

// TODO: remove all 'this' references

namespace control {
	Controller::Controller()
	{
		ROS_INFO("Running controller");
		model_name_ = "anymal"; // TODO: Replace with node argument
		InitRosTopics();
		SpinRosThreads();
		CreateStandupTrajectory();

		InitController();
		controller_initialized_ = true;

		ros::Duration(0.5).sleep(); // Wait before starting

		RunStandupSequence();
	}

	Controller::~Controller()
	{
			ros_node_->shutdown();

			ros_process_queue_.clear();
			ros_process_queue_.disable();
			ros_process_queue_thread_.join();

			ros_publish_queue_.clear();
			ros_publish_queue_.disable();
			ros_publish_queue_thread_.join();
	}

	void Controller::InitController()
	{
		SetStartTime();

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
		const std::vector<double> breaks = { 0.0, 5.0, 10.0 };
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
		samples.push_back(touchdown_conf);

		// Use drake piecewise polynomials for easy trajectory construction
		standup_pos_traj_ =
			drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
					breaks, samples
					);
		standup_vel_traj_ = standup_pos_traj_.derivative(1);
	}

	void Controller::RunController()
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

		CalcTorqueCmd();
	}

	void Controller::CalcFeetTrackingError()
	{
		feet_pos_error_ =
			standup_pos_traj_.value(t_)
			- robot_dynamics_.GetFeetPositions(q_);

		feet_vel_ff_ = standup_vel_traj_.value(t_);
	}

	// TODO: Move this into its own ROS node
	void Controller::CalcTorqueCmd()
	{
		Eigen::Matrix<double,12,1> tau_cmd;
		tau_cmd.setZero();

		tau_cmd = k_joints_p_ * (q_j_cmd_ - q_.block<12,1>(7,0))
			+ k_joints_d_ * (q_j_dot_cmd_ - u_.block<12,1>(6,0));
					
		std_msgs::Float64MultiArray torque_cmd_msg;
		tf::matrixEigenToMsg(tau_cmd, torque_cmd_msg);

		this->torque_cmd_pub_.publish(torque_cmd_msg);
	}

	// *** //
	// ROS //
	// *** //

	void Controller::InitRosTopics()
	{
		if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(
            argc,
            argv,
            "controller_node",
            ros::init_options::NoSigintHandler
        );
    }
		ros_node_.reset(new ros::NodeHandle("controller_node"));

		// Set up advertisements
		ros::AdvertiseOptions pos_cmd_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_name_ + "/pos_cmd",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		ros::AdvertiseOptions vel_cmd_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_name_ + "/vel_cmd",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		ros::AdvertiseOptions torque_cmd_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_name_ + "/torque_cmd",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		this->pos_cmd_pub_ = this->ros_node_->advertise(pos_cmd_ao);
		this->vel_cmd_pub_ = this->ros_node_->advertise(vel_cmd_ao);
		this->torque_cmd_pub_ = this->ros_node_->advertise(torque_cmd_ao);

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

		gen_coord_sub_ = ros_node_->subscribe(gen_coord_so);
		gen_vel_sub_ = ros_node_->subscribe(gen_vel_so);

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
		static const double timeout = 0.01;
		while (this->ros_node_->ok())
		{
			this->ros_process_queue_.callAvailable(
					ros::WallDuration(timeout)
					);
		}
	}

	void Controller::PublishQueueThread()
	{
		ros::Rate loop_rate(10);

		while (this->ros_node_->ok())
		{
			if (!controller_initialized_)
				continue;

			RunController();
			loop_rate.sleep();	
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


	// ****************
// TODO: REMOVE
	// ****************
	//
	//
	//
	//
	void Controller::PublishJointVelCmd(double vel_cmd)
	{
			Eigen::Matrix<double,12,1> q_cmd;
			q_cmd.setZero();
			q_cmd(2) = vel_cmd;
			q_cmd(5) = vel_cmd;
			q_cmd(8) = vel_cmd;
			q_cmd(11) = vel_cmd;

			std_msgs::Float64MultiArray pos_cmd_msg;
			tf::matrixEigenToMsg(q_cmd, pos_cmd_msg);

			this->vel_cmd_pub_.publish(pos_cmd_msg);
	}
	// TODO: Remove
	void Controller::PublishIdlePositionCmd()
	{
			Eigen::Matrix<double,12,1> q_cmd;
			for (int i = 0; i < 12; ++i)
			{
				q_cmd(i) = 0.0;
			}

			std_msgs::Float64MultiArray pos_cmd_msg;
			tf::matrixEigenToMsg(q_cmd, pos_cmd_msg);

			this->pos_cmd_pub_.publish(pos_cmd_msg);
			ROS_INFO("Published idle joint position command");
	}

	// TODO: Remove
	void Controller::PublishTestTorqueCmd()
	{
			Eigen::Matrix<double,12,1> tau_cmd;
			for (int i = 0; i < 12; ++i)
			{
				tau_cmd(i) = 20.0;
			}

			std_msgs::Float64MultiArray torque_cmd_msg;
			tf::matrixEigenToMsg(tau_cmd, torque_cmd_msg);

			this->torque_cmd_pub_.publish(torque_cmd_msg);
			ROS_INFO("Published test torques");
	}
}
