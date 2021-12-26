#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include "anymal_constants.hpp"

class JointStatePublisher
{
	public:
		JointStatePublisher();
		JointStatePublisher(int frequency_);

	private:
		ros::NodeHandle node_handle_;
		ros::Rate loop_rate_;

		ros::Publisher joint_state_pub_;
		ros::Subscriber joint_positions_sub_; // For now, this only reads positions. Expand to velocities and efforts when needed.

		Eigen::Matrix<double, kNumJoints, 1> joint_positions_;

		void InitRos();
		void PublishJointState();
		void OnJointPosMsg(
				const std_msgs::Float64MultiArrayConstPtr &msg
				);
		void SetJointPositions(
				const std::vector<double> &received_pos
				);
};

