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
			std::string model_name_;
			const int kNumGenCoords_ = 19; // TODO: Not currently used everywhere
			Eigen::Matrix<double, 19, 1> q_;
			Dynamics robot_dynamics_;
			
			int n_legs_ = 4;
			int n_dims_ = 3;
			double swing_height_ = 0.2;

			// **************** //
			// STANDUP SEQUENCE //
			// **************** //

			drake::trajectories::PiecewisePolynomial<double>
				standup_pos_traj_;
			drake::trajectories::PiecewisePolynomial<double>
				standup_vel_traj_;
			void CreateStandupTrajectory();
			void RunStandupSequence();


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

			// **************** //
			// HELPER FUNCTIONS //
			// **************** //
			Eigen::MatrixXd CalcPseudoInverse(Eigen::MatrixXd A);

			// TODO: Remove
			void PublishIdlePositionCmd();
			void PublishTestTorqueCmd();
	};
}
