#pragma once

#include "dynamics/dynamics.hpp"
#include "control/integrator.hpp"
#include "anymal_constants.hpp" // TODO Use this everywhere instead of numbers

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

			gen_coord_vector_t q_;
			gen_vel_vector_t u_;
			Eigen::MatrixXd q_j_;
			Eigen::MatrixXd q_j_dot_;

			Dynamics robot_dynamics_;
			ros::Time start_time_;
			bool controller_ready_ = false;
			
			void SetStateToCmd();

			// **************** //
			// STANDUP SEQUENCE //
			// **************** //

			double seconds_to_initial_config_ = 2.5; 
			double seconds_to_standup_config_ = 10.0;
			double standing_height_ = 0.6;

			drake::trajectories::PiecewisePolynomial<double>
				q_j_initial_config_traj_;
			drake::trajectories::PiecewisePolynomial<double>
				q_j_dot_initial_config_traj_;

			drake::trajectories::PiecewisePolynomial<double>
				feet_standup_pos_traj_;
			drake::trajectories::PiecewisePolynomial<double>
				feet_standup_vel_traj_;

			bool created_standup_traj_ = false;

			void CreateInitialConfigTraj();
			void CreateStandupTraj();

			void CalcJointCmd();

			// ********** //
			// CONTROLLER //
			// ********** //

			double t_ = 0;

			ros::Rate loop_rate_;

			double k_pos_p_ = 0.001; // TODO: Tune these

			Eigen::Matrix<double,12,1> feet_pos_error_;
			Eigen::Matrix<double,12,1> feet_vel_ff_;

			Eigen::MatrixXd J_feet_pos_;

			void InitController();
			void JacobianController(); // TODO: Rename

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

			joint_vector_t GetJointsPos();
			joint_vector_t GetJointsVel();
			void SetGenCoords(const std::vector<double> &gen_coords);
			void SetGenVels(const std::vector<double> &gen_vels);

			// **************** //
			// HELPER FUNCTIONS //
			// **************** //

			drake::trajectories::PiecewisePolynomial<double>
				CreateFirstOrderHoldTraj(
						std::vector<double> breaks,
						std::vector<Eigen::MatrixXd> samples
						);
			void SetStateVariablesToZero();
			void SetStartTime();
			double GetElapsedTimeSince(ros::Time t);
			Eigen::MatrixXd CalcPseudoInverse(Eigen::MatrixXd A);
	};
}
