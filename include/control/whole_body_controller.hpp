#pragma once

#include "dynamics/dynamics.hpp"
#include "control/integrator.hpp"
#include "control/ho_qp/ho_qp_controller.hpp"
#include "anymal_constants.hpp"
#include "math.hpp"

#include <thread>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <drake/common/trajectories/piecewise_polynomial.h>

#include <Eigen/Core>


namespace control
{
	class WholeBodyController 
	{
		public:
			WholeBodyController(int frequency);
			~WholeBodyController();

		private:
			bool print_frequency_ = false;

			ros::NodeHandle ros_node_;
			const std::string model_name_ = "anymal";

			// ************* //
			// INIT SEQUENCE //
			// ************* //

			ros::Time init_start_time_;
			drake::trajectories::PiecewisePolynomial<double>
				q_j_init_traj_;
			drake::trajectories::PiecewisePolynomial<double>
				q_j_dot_init_traj_;
			const double init_sequence_duration_ = 3.0;
			void CreateInitialJointConfigTraj();

			// ********** //
			// CONTROLLER //
			// ********** //

			enum ControlMode {
				kJointTracking,
				kHoQpController
			} control_mode_;

			gen_coord_vector_t q_;
			gen_vel_vector_t u_;

			Eigen::VectorXd r_cmd_;
			Eigen::VectorXd r_dot_cmd_;
			Eigen::VectorXd r_ddot_cmd_;

			Eigen::VectorXd r_c_cmd_;
			Eigen::VectorXd r_c_dot_cmd_;
			Eigen::VectorXd r_c_ddot_cmd_;
			Eigen::VectorXd contact_pattern_cmd_;

			Eigen::VectorXd q_j_;
			Eigen::VectorXd q_j_dot_;

			Eigen::VectorXd q_j_cmd_;
			Eigen::VectorXd q_j_dot_cmd_;
			Eigen::VectorXd q_j_ddot_cmd_;
			Eigen::VectorXd tau_j_cmd_;

			bool controller_ready_ = false;
			HoQpController ho_qp_controller_;

			void SetControlMode(ControlMode target_mode);
			void UpdateJointCommand();
			void DirectJointControl();

			// *********** //
			// INTEGRATORS //
			// *********** //

			Integrator q_j_dot_cmd_integrator_;
			Integrator q_j_ddot_cmd_integrator_;

			void InitIntegrators();
			void SetIntegratorsToCurrentState();
			void SetJointCmdFromIntegrators();

			// *** //
			// ROS //
			// *** //

			ros::Rate loop_rate_;
			ros::Time last_update_;
			bool received_first_state_msg_ = false;

			ros::Publisher q_j_cmd_pub_;
			ros::Publisher q_j_dot_cmd_pub_;
			ros::Publisher tau_j_cmd_pub_;

			ros::Subscriber gen_coord_sub_;
			ros::Subscriber gen_vel_sub_;

			ros::Subscriber base_pos_cmd_sub_;
			ros::Subscriber base_vel_cmd_sub_;
			ros::Subscriber base_acc_cmd_sub_;

			ros::Subscriber legs_pos_cmd_sub_;
			ros::Subscriber legs_vel_cmd_sub_;
			ros::Subscriber legs_acc_cmd_sub_;
			ros::Subscriber legs_contact_cmd_sub_;

			ros::CallbackQueue ros_process_queue_;
			ros::CallbackQueue ros_publish_queue_;
			std::thread ros_process_queue_thread_;
			std::thread ros_publish_queue_thread_;

			void SetupRosTopics();
			void SetupJointCmdAdvertisement();
			void SetupStateSubscriptions();

			void SetupBaseCmdSubscriptions();
			void SetupLegCmdSubscriptions();

			void SpinRosThreads();
			void PublishQueueThread();
			void ProcessQueueThread();

			void PublishJointPosCmd();
			void PublishJointVelCmd();
			void PublishJointTorqueCmd();

			void OnGenCoordMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnGenVelMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);

			void OnBasePosCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnBaseVelCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnBaseAccCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnLegsPosCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnLegsVelCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnLegsAccCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnLegsContactCmdMsg(
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

			void SetBasePosCmd(const std::vector<double> &base_pos_cmd);
			void SetBaseVelCmd(const std::vector<double> &base_vel_cmd);
			void SetBaseAccCmd(const std::vector<double> &base_acc_cmd);

			void SetLegPosCmd(const std::vector<double> &leg_pos_cmd);
			void SetLegVelCmd(const std::vector<double> &leg_vel_cmd);
			void SetLegAccCmd(const std::vector<double> &leg_acc_cmd);
			void SetLegContactCmd(const std::vector<double> &leg_contact_cmd);

			// **************** //
			// HELPER FUNCTIONS //
			// **************** //

			void WaitForPublishedTime();
			void WaitForPublishedState();

			void SetVariablesToZero();
			void SetZeroBaseCmdMotion();
			void SetZeroLegCmdMotion();
			double GetElapsedTimeSince(ros::Time t);
	};
}
