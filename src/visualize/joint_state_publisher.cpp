#include "visualize/joint_state_publisher.hpp"

JointStatePublisher::JointStatePublisher()
	: JointStatePublisher(100)
{}

JointStatePublisher::JointStatePublisher(int frequency_)
	: loop_rate_(frequency_)
{
	ROS_INFO("Starting joint state visualizer at %d Hz", frequency_);

	joint_positions_.setZero();
	InitRos();
}

void JointStatePublisher::InitRos()
{
	joint_state_pub_ = node_handle_
		.advertise<sensor_msgs::JointState>(
				"/joint_states", 1
				);

	joint_positions_sub_ = node_handle_
		.subscribe<std_msgs::Float64MultiArray>(
				"/joint_positions", 1,
				boost::bind(&JointStatePublisher::OnJointPosMsg, this, _1)
				);

	// Single threaded
	while(ros::ok())
	{
		ros::spinOnce(); // fetch messages
		PublishJointState();
		ros::spinOnce(); // publish messages
		loop_rate_.sleep();
	}
}

void JointStatePublisher::PublishJointState()
{
	sensor_msgs::JointState joint_state_msg;
	joint_state_msg.header.stamp = ros::Time::now();

	for(int joint_i = 0; joint_i < kNumJoints; joint_i++)
	{
			joint_state_msg.name
				.push_back(kJointNames[joint_i]);
			joint_state_msg.position
				.push_back(joint_positions_(joint_i));
	}

	joint_state_pub_.publish(joint_state_msg);
}

void JointStatePublisher::OnJointPosMsg(
		const std_msgs::Float64MultiArrayConstPtr &msg
		)
{
	SetJointPositions(msg->data);
}

void JointStatePublisher::SetJointPositions(
		const std::vector<double> &received_pos
		)
{
	for (int joint_i = 0; joint_i < kNumJoints; ++joint_i)
		joint_positions_(joint_i) = received_pos[joint_i];
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_state_publisher",
		ros::init_options::NoSigintHandler
		);
	int frequency = 100;
	JointStatePublisher joint_state_publisher_(100);
	return 0;
}
