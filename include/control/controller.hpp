#pragma once

#include <thread>
#include "ros/ros.h"
#include "ros/console.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "std_msgs/Float64MultiArray.h"
#include "eigen_conversions/eigen_msg.h"

#include <Eigen/Core>

namespace control
{
	class Controller
	{
		public:
			Controller();
			~Controller();

		private:
			std::string model_name_;

			std::unique_ptr<ros::NodeHandle> ros_node_;

			// Advertisements
			ros::Publisher pos_cmd_pub_;
			ros::Publisher vel_cmd_pub_;
			ros::Publisher torque_cmd_pub_;

			// Queues and their threads
			ros::CallbackQueue ros_process_queue_;
			ros::CallbackQueue ros_publish_queue_;
			std::thread ros_process_queue_thread_;
			std::thread ros_publish_queue_thread_;

			void InitRosTopics();

			void PublishQueueThread();
			void ProcessQueueThread();

			void PublishIdlePositionCmd();
			void PublishTestTorqueCmd();
	};
}
