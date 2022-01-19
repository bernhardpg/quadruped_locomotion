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
	class WholeBodyController // TODO: Is this really more of a state machine?
	{
		public:
			WholeBodyController(int frequency);
			~WholeBodyController();

		private:
			bool print_frequency_ = false;

			ros::NodeHandle ros_node_;
			std::string model_name_;

			Dynamics robot_dynamics_;

			// **************** //
			// STANDUP SEQUENCE //
			// **************** //

			const double seconds_to_initial_config_ = 3.0; 
			const double seconds_to_standup_config_ = 2.0;
			double traj_end_time_s_ = 2.0;
			double standing_height_ = 0.6;

			void SetJointInitialConfigTraj();
			void SetComStandupTraj();
			void SetDanceTraj();

			// ********** //
			// CONTROLLER //
			// ********** //

			gen_coord_vector_t q_;
			gen_vel_vector_t u_;

			Eigen::VectorXd r_cmd_;
			Eigen::VectorXd r_dot_cmd_;
			Eigen::VectorXd r_ddot_cmd_;

			Eigen::VectorXd r_c_cmd_;
			Eigen::VectorXd r_c_dot_cmd_;
			Eigen::VectorXd r_c_ddot_cmd_;
			std::vector<int> legs_in_contact_;

			Eigen::MatrixXd q_j_;
			Eigen::MatrixXd q_j_dot_;

			Eigen::Matrix<double,12,1> q_j_cmd_;
			Eigen::Matrix<double,12,1> q_j_dot_cmd_;
			Eigen::Matrix<double,12,1> q_j_ddot_cmd_;
			Eigen::Matrix<double,12,1> tau_j_cmd_;

			Integrator q_j_dot_cmd_integrator_;
			Integrator q_j_ddot_cmd_integrator_;

			enum ControlMode {
				kJointTracking,
				kFeetTracking,
				kSupportConsistentTracking,
				kHoQpController
			} control_mode_;

			drake::trajectories::PiecewisePolynomial<double>
				q_j_cmd_traj_;
			drake::trajectories::PiecewisePolynomial<double>
				q_j_dot_cmd_traj_;

			drake::trajectories::PiecewisePolynomial<double>
				feet_cmd_pos_traj_;
			drake::trajectories::PiecewisePolynomial<double>
				feet_cmd_vel_traj_;

			Eigen::MatrixXd J_task_;
			drake::trajectories::PiecewisePolynomial<double>
				task_vel_traj_;

			ros::Rate loop_rate_;

			double k_pos_p_ = 1.0;
			bool controller_ready_ = false;

			HoQpController ho_qp_controller_;

			void UpdateJointCommand();
			void DirectJointControl();
			void SupportConsistentControl();

			void IntegrateJointAccelerations();

			// ************* //
			// STATE MACHINE // 
			// ************* //

			enum RobotMode {
				kIdle, kStandup, kWalk, kDance
			} robot_mode_;
			
			ros::Time mode_start_time_;
			double seconds_in_mode_ = 0;
			ros::Time last_update_;

			void SetRobotMode(RobotMode target_mode);
			void SetModeStartTime();

			// *** //
			// ROS //
			// *** //

			bool received_first_state_msg_ = false;

			// Services
			ros::ServiceServer cmd_standup_service_;	
			ros::ServiceServer cmd_dance_service_;	
			ros::ServiceServer cmd_walk_service_;	

			// Advertisements
			ros::Publisher q_j_cmd_pub_;
			ros::Publisher q_j_dot_cmd_pub_;
			ros::Publisher tau_j_cmd_pub_;

			// Subscriptions
			ros::Subscriber gen_coord_sub_;
			ros::Subscriber gen_vel_sub_;

			ros::Subscriber com_pos_cmd_sub_;
			ros::Subscriber com_vel_cmd_sub_;
			ros::Subscriber com_acc_cmd_sub_;

			ros::Subscriber legs_pos_cmd_sub_;
			ros::Subscriber legs_vel_cmd_sub_;
			ros::Subscriber legs_acc_cmd_sub_;
			ros::Subscriber legs_contact_cmd_sub_;

			// Queues and their threads
			ros::CallbackQueue ros_process_queue_;
			ros::CallbackQueue ros_publish_queue_;
			std::thread ros_process_queue_thread_;
			std::thread ros_publish_queue_thread_;

			void SetupRosTopics();
			void SetupJointCmdAdvertisement();
			void SetupStateSubscriptions();

			void SetupComCmdSubscriptions();
			void SetupLegCmdSubscriptions();

			void SetupRosServices();
			void SpinRosThreads();
			void PublishQueueThread();
			void ProcessQueueThread();

			void PublishJointPosCmd();
			void PublishJointVelCmd();
			void PublishJointTorqueCmd();

			bool CmdStandupService(
							const std_srvs::Empty::Request &_req,
							std_srvs::Empty::Response &_res
					);
			bool CmdDanceService(
							const std_srvs::Empty::Request &_req,
							std_srvs::Empty::Response &_res
					);
			bool CmdWalkService(
							const std_srvs::Empty::Request &_req,
							std_srvs::Empty::Response &_res
					);

			void OnGenCoordMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnGenVelMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);

			void OnComPosCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnComVelCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnComAccCmdMsg(
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

			void SetComPosCmd(const std::vector<double> &com_pos_cmd);
			void SetComVelCmd(const std::vector<double> &com_vel_cmd);
			void SetComAccCmd(const std::vector<double> &com_acc_cmd);

			void SetLegPosCmd(const std::vector<double> &leg_pos_cmd);
			void SetLegVelCmd(const std::vector<double> &leg_vel_cmd);
			void SetLegAccCmd(const std::vector<double> &leg_acc_cmd);
			void SetLegContactCmd(const std::vector<double> &leg_contact_cmd);

			// **************** //
			// HELPER FUNCTIONS //
			// **************** //

			Eigen::MatrixXd EvalPosTrajAtTime(
					drake::trajectories::PiecewisePolynomial<double> traj,
					double curr_time);
			Eigen::MatrixXd EvalVelTrajAtTime(
					drake::trajectories::PiecewisePolynomial<double> traj,
					double curr_time);

			void WaitForPublishedTime();
			void WaitForPublishedState();

			drake::trajectories::PiecewisePolynomial<double>
				CreateFirstOrderHoldTraj(
						std::vector<double> breaks,
						std::vector<Eigen::MatrixXd> samples
						);
			void SetVariablesToZero();
			void SetZeroComCmdMotion();
			void SetZeroLegCmdMotion();
			double GetElapsedTimeSince(ros::Time t);
	};
}
