#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include <dynamic_reconfigure/server.h>
#include <quadruped_locomotion/DynamicControlConfig.h>
#include "anymal_constants.hpp"

namespace control
{
	class JointController
	{
		public:
			JointController();
			JointController(int frequency_);

		private:
			int frequency_;

			std::string model_name_;
			bool received_cmd_ = false;

			Eigen::Matrix<double, kNumJoints, 1> q_j_;
			Eigen::Matrix<double, kNumJoints, 1> q_j_dot_;
			Eigen::Matrix<double, kNumJoints, 1> q_j_cmd_;
			Eigen::Matrix<double, kNumJoints, 1> q_j_dot_cmd_;
			Eigen::Matrix<double, kNumJoints, 1> tau_j_ff_cmd_;
			Eigen::Matrix<double, kNumJoints, 1> tau_cmd_;

			double k_joints_p_= 100; 
			double k_joints_d_ = 5;

			void CalcJointTorques();

			// *** //
			// ROS //
			// *** //

			void InitRos();

			ros::NodeHandle node_handle_;
			ros::Rate loop_rate_;

			ros::Publisher torque_cmd_pub_;
			ros::Subscriber q_joint_cmd_sub_;
			ros::Subscriber q_joint_dot_cmd_sub_;
			ros::Subscriber tau_joint_cmd_sub_;
			ros::Subscriber gen_coord_sub_;
			ros::Subscriber gen_vel_sub_;

			void OnGenCoordMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnGenVelMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnJointPosCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnJointVelCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnJointTorqueCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);

			// Dynamic reconfiguration of parameters
			dynamic_reconfigure::Server
				<quadruped_locomotion::DynamicControlConfig>
				dc_server_;
			dynamic_reconfigure::Server
				<quadruped_locomotion::DynamicControlConfig>
				::CallbackType dc_callback_;

			void InitDynamicReconfigureRos();
			void DynamicConfigureCallback(
					quadruped_locomotion::DynamicControlConfig &config,
					uint32_t level
					);
	};
}

