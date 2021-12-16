#include "control/controller.hpp"

namespace control {
	Controller::Controller()
	{
		ROS_INFO("Running controller");

		this->model_name_ = "anymal"; // TODO: Replace with node argument

		this->InitRosTopics();

		// TODO: Just a testline, remove!
		//auto t = prog_.NewIndeterminates(1,1,"t");

	}

	Controller::~Controller()
	{
			this->ros_node_->shutdown();

			this->ros_process_queue_.clear();
			this->ros_process_queue_.disable();
			this->ros_process_queue_thread_.join();

			this->ros_publish_queue_.clear();
			this->ros_publish_queue_.disable();
			this->ros_publish_queue_thread_.join();
	}

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
		this->ros_node_.reset(new ros::NodeHandle("controller_node"));

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

	void Controller::PublishQueueThread()
	{
		ros::Rate loop_rate(1);

		// TODO: Wait 10 seconds before publishing controller inputs
		ros::Duration(10).sleep();

		while (this->ros_node_->ok())
		{
			this->PublishIdlePositionCmd();
			loop_rate.sleep();	
		}
	}

}
