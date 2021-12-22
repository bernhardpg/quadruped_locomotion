#include "control/controller.hpp"

// TODO: remove all 'this' references

namespace control {
	Controller::Controller()
	{
		ROS_INFO("Running controller");
		model_name_ = "anymal"; // TODO: Replace with node argument
		InitRosTopics();
		SpinRosThreads();
		SetStartTime();
		CreateStandupTrajectory();
		ros::Duration(0.5).sleep(); // Wait before starting

		controller_initialized_ = true;
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

	// **************** //
	// STANDUP SEQUENCE //
	// **************** //

	void Controller::RunStandupSequence()
	{
		ROS_INFO("Starting standup sequence");
		Eigen::MatrixXd J_full = robot_dynamics_.GetFullContactJacobian(q_);
		CalcPseudoInverse(J_full);
	}

	void Controller::CreateStandupTrajectory()
	{
		// Three standup steps 
		const std::vector<double> breaks = { 0.0, 5.0, 10.0 };
		std::vector<Eigen::MatrixXd> samples;

		Eigen::MatrixXd start_conf = Eigen::MatrixXd(n_dims_, n_legs_);
		start_conf = robot_dynamics_.GetFeetPositions(q_);

		Eigen::MatrixXd neutral_conf = Eigen::MatrixXd(n_dims_, n_legs_);
		Eigen::Matrix<double, 19, 1> q_neutral;
		q_neutral.block<7,1>(0,0) = q_.block<7,1>(0,0);
		q_neutral.block<12,1>(7,0) = Eigen::VectorXd::Zero(12);
		neutral_conf = robot_dynamics_.GetFeetPositions(q_neutral);

		Eigen::MatrixXd apex_conf = neutral_conf;
		for (int leg_i = 0; leg_i < n_legs_; ++leg_i)
			apex_conf(2, leg_i) = swing_height_;

		Eigen::MatrixXd touchdown_conf = neutral_conf;
		for (int leg_i = 0; leg_i < n_legs_; ++leg_i)
			touchdown_conf(2, leg_i) = 0;

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

	void Controller::PositionController()
	{
		double t = GetElapsedTime();

		Eigen::MatrixXd pos_error;
		pos_error.setZero();
		Eigen::MatrixXd curr_pos = robot_dynamics_.GetFeetPositions(q_);
		pos_error = standup_pos_traj_.value(t) - curr_pos;
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

		gen_coord_sub_ = ros_node_->subscribe(gen_coord_so);

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

			PositionController();
			loop_rate.sleep();	
		}
	}

	void Controller::OnGenCoordMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetGenCoords(msg->data);
	}

	// ***************** //
	// SETTERS & GETTERS //
	// ***************** //

	void Controller::SetGenCoords(const std::vector<double> &gen_coords)
	{
		for (int i = 0; i < kNumGenCoords_; ++i)
			q_(i) = gen_coords[i];
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

	double Controller::GetElapsedTime()
	{
		double elapsed_time = (ros::Time::now() - start_time_).toSec();
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
