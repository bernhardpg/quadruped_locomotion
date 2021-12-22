#pragma once

#include "dynamics/dynamics.hpp"

#include <thread>
#include "ros/ros.h"
#include "ros/console.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "std_msgs/Float64MultiArray.h"
#include "eigen_conversions/eigen_msg.h"

#include <drake/solvers/mathematical_program.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

#include <Eigen/Core>


namespace control
{
	class Controller
	{
		public:
			Controller();
			~Controller();

		private:
			Dynamics robot_dynamics_;
			const int kNumGenCoords_ = 19;
			Eigen::Matrix<double, 19, 1> q_;
			
			int n_legs_ = 4;
			int n_dims_ = 3;
			double swing_height_ = 0.2;

			Eigen::Vector3d LF_KFE_pos_;
			Eigen::Vector3d LH_KFE_pos_;
			Eigen::Vector3d RH_KFE_pos_;
			Eigen::Vector3d RF_KFE_pos_;

			void RunStandupSequence();

			std::string model_name_;
			void PublishIdlePositionCmd();
			void PublishTestTorqueCmd();

			// *** //
			// ROS //
			// *** //
			std::unique_ptr<ros::NodeHandle> ros_node_;

			// Advertisements
			ros::Publisher pos_cmd_pub_;
			ros::Publisher vel_cmd_pub_;
			ros::Publisher torque_cmd_pub_;

			// Subscriptions
			ros::Subscriber gen_coord_sub_;

			// Queues and their threads
			ros::CallbackQueue ros_process_queue_;
			ros::CallbackQueue ros_publish_queue_;
			std::thread ros_process_queue_thread_;
			std::thread ros_publish_queue_thread_;

			void InitRosTopics();
			void PublishQueueThread();
			void ProcessQueueThread();

			void OnGenCoordMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);

	};
}
