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

		ros::Publisher gen_coord_pub_;
		ros::Publisher gen_vel_pub_;
		ros::Publisher joint_state_pub_;

		ros::Subscriber joint_positions_sub_; 
		ros::Subscriber joint_velocities_sub_; // TODO: Expand to efforts when needed.

		// will feedback commanded joint ppsitions as state
		bool publish_cmd_as_state_ = true;
		joint_vector_t joint_positions_;
		joint_vector_t joint_velocities_;

		void InitRos();
		void PublishJointState();
		void PublishGenCoords();
		void PublishGenVels();
		Eigen::Matrix<double, 7, 1> GetDefaultBodyPose();
		void OnJointPosMsg(
				const std_msgs::Float64MultiArrayConstPtr &msg
				);
		void OnJointVelMsg(
				const std_msgs::Float64MultiArrayConstPtr &msg
				);
		void SetJointPositions(
				const std::vector<double> &received_pos
				);
		void SetJointVelocities(
				const std::vector<double> &received_vel
				);
};

