#pragma once

#include "dynamics/dynamics.hpp"
#include "control/integrator.hpp"

#include <thread>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <drake/solvers/mathematical_program.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

#include <Eigen/Core>


namespace control
{
	class Controller // TODO: Rename this class, it will not actually be a controller
	{
		public:
			Controller(int frequency);
			~Controller();

		private:
			ros::NodeHandle ros_node_;

			std::string model_name_;
			const int kNumGenCoords_ = 19; // TODO: Not currently used everywhere
			const int kNumJoints_ = 12; // TODO: Not currently used everywhere
			Eigen::Matrix<double, 19, 1> q_;
			Eigen::Matrix<double, 18, 1> u_;
			Dynamics robot_dynamics_;
			ros::Time start_time_;
			bool controller_ready_ = false;
			
			int n_legs_ = 4;
			int n_dims_ = 3;
			double swing_height_ = 0.6;

			// **************** //
			// STANDUP SEQUENCE //
			// **************** //

			double standup_start_time_ = 0.0;
			double standup_end_time_ = 2.5;

			drake::trajectories::PiecewisePolynomial<double>
				standup_pos_traj_;
			drake::trajectories::PiecewisePolynomial<double>
				standup_vel_traj_;

			drake::trajectories::PiecewisePolynomial<double>
				q_j_ref_traj_;
			drake::trajectories::PiecewisePolynomial<double>
				q_j_dot_ref_traj_;

			void CreateStandupTrajectory();
			void RunStandupSequence();

			void CalcJointCmd();
			void CreateStandupJointTraj();

			// ********** //
			// CONTROLLER //
			// ********** //

			double t_ = 0;

			ros::Rate loop_rate_;

			double k_pos_p_ = 0.1; // TODO: Tune these

			Eigen::Matrix<double,12,1> feet_pos_error_;
			Eigen::Matrix<double,12,1> feet_vel_ff_;

			Eigen::MatrixXd J_feet_pos_;

			void InitController();
			void JacobianController(); // TODO: Rename

			void CalcFeetTrackingError();

			Eigen::Matrix<double,12,1> q_j_cmd_;
			Eigen::Matrix<double,12,1> q_j_dot_cmd_;

			Integrator q_j_dot_cmd_integrator_;

			// *** //
			// ROS //
			// *** //

			// Advertisements
			ros::Publisher q_j_cmd_pub_;
			ros::Publisher q_j_dot_cmd_pub_;

			// Subscriptions
			ros::Subscriber gen_coord_sub_;
			ros::Subscriber gen_vel_sub_;

			// Queues and their threads
			ros::CallbackQueue ros_process_queue_;
			ros::CallbackQueue ros_publish_queue_;
			std::thread ros_process_queue_thread_;
			std::thread ros_publish_queue_thread_;

			void InitRos();
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
	};
}
