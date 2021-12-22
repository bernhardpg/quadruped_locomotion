#include "control/controller.hpp"

// TODO: remove all 'this' references

namespace control {
	Controller::Controller()
	{
		ROS_INFO("Running controller");
		model_name_ = "anymal"; // TODO: Replace with node argument
		InitRosTopics();

		ros::Duration(5).sleep();
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

		// TODO: Move these
		LF_KFE_pos_ << 0.36, -0.29, 0.28; 
		LH_KFE_pos_ << -0.36, -0.29, 0.28; 
		RH_KFE_pos_ << -0.36, 0.29, 0.28; 
		RF_KFE_pos_ << 0.36, 0.29, 0.28; 

		// Construct trajectory for standing up
		const std::vector<double> breaks = { 0.0, 5.0, 10.0 };
		std::vector<Eigen::MatrixXd> samples;

		auto start_conf = Eigen::MatrixXd(n_dims_, n_legs_);
		auto apex_conf = Eigen::MatrixXd(n_dims_, n_legs_);
		auto touchdown_conf = Eigen::MatrixXd(n_dims_, n_legs_);

		start_conf = robot_dynamics_.GetFeetPositions(q_);

		apex_conf <<
			LF_KFE_pos_(0), LH_KFE_pos_(0), RH_KFE_pos_(0), RF_KFE_pos_(0), 
			LF_KFE_pos_(1), LH_KFE_pos_(1), RH_KFE_pos_(1), RF_KFE_pos_(1), 
			swing_height_, swing_height_, swing_height_, swing_height_;

		touchdown_conf <<
			LF_KFE_pos_(0), LH_KFE_pos_(0), RH_KFE_pos_(0), RF_KFE_pos_(0), 
			LF_KFE_pos_(1), LH_KFE_pos_(1), RH_KFE_pos_(1), RF_KFE_pos_(1), 
			0, 0, 0, 0;

		samples.push_back(start_conf);
		samples.push_back(apex_conf);
		samples.push_back(touchdown_conf);

		const auto feet_pos_traj =
			drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
					breaks, samples
					);


		const auto feet_vel_traj = feet_pos_traj.derivative(1);
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

		// Spin up the queue helper threads
		this->ros_process_queue_thread_ = std::thread(
				std::bind(&Controller::ProcessQueueThread, this)
				);
		this->ros_publish_queue_thread_ = std::thread(
				std::bind(&Controller::PublishQueueThread, this)
				);

		ROS_INFO("Finished setting up ros topics");
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
		ros::Rate loop_rate(1);

		while (this->ros_node_->ok())
		{
			loop_rate.sleep();	
		}
	}

	void Controller::OnGenCoordMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		for (int i = 0; i < kNumGenCoords_; ++i)
			q_(i) = msg->data[i];
	}
}
