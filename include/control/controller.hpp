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
			Eigen::Matrix<double, 18, 1> u_;
			Dynamics robot_dynamics_;
			ros::Time start_time_;
			bool controller_initialized_ = false;
			
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

			void RunController();

			Eigen::VectorXd integral_;
			double t_ = 0;
			double dt_ = 0;
			ros::Time last_run_; // TODO: change name

			Eigen::Matrix<double,12,1> q_j_dot_cmd_;
			Eigen::Matrix<double,12,1> q_j_cmd_;

			void CalcTorqueCmd(); // TODO: cleanup
			void Integrate(Eigen::VectorXd vec); // TODO: cleanup, make this able to integrate anything
	
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
			ros::Subscriber gen_vel_sub_;

			// Queues and their threads
			ros::CallbackQueue ros_process_queue_;
			ros::CallbackQueue ros_publish_queue_;
			std::thread ros_process_queue_thread_;
			std::thread ros_publish_queue_thread_;

			void InitRosTopics();
			void SpinRosThreads();
			void PublishQueueThread();
			void ProcessQueueThread();

			void OnGenCoordMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnGenVelMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void PublishJointVelCmd(double vel_cmd);

			// ***************** //
			// SETTERS & GETTERS //
			// ***************** //

			void SetGenCoords(const std::vector<double> &gen_coords);
			void SetGenVels(const std::vector<double> &gen_vels);

			// **************** //
			// HELPER FUNCTIONS //
			// **************** //

			void SetStartTime();
			double GetElapsedTimeSince(ros::Time t);
			Eigen::MatrixXd CalcPseudoInverse(Eigen::MatrixXd A);

			// TODO: Remove
			void PublishIdlePositionCmd();
			void PublishTestTorqueCmd();
	};
}
