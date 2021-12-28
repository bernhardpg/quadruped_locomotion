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
	gen_coord_pub_ = node_handle_
		.advertise<std_msgs::Float64MultiArray>(
				"/anymal/gen_coord/", 1
				);

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

		if (publish_cmd_as_state_)
			PublishGenCoords();

		PublishJointState();
		ros::spinOnce(); // publish messages
		loop_rate_.sleep();
	}
}

void JointStatePublisher::PublishGenCoords()
{
	joint_vector_t q_j; // generalized coordinates
	q_j = joint_positions_;

	gen_coord_vector_t q;
	q.block<7,1>(0,0) = GetDefaultBodyPose();
	q.block<kNumJoints,1>(kNumPoseCoords,0) = q_j;

	std_msgs::Float64MultiArray gen_coord_msg;
	tf::matrixEigenToMsg(q, gen_coord_msg);

	gen_coord_pub_.publish(gen_coord_msg);
}

Eigen::Matrix<double, 7, 1> JointStatePublisher::GetDefaultBodyPose()
{
	Eigen::Matrix<double, 7, 1> q_b;
	q_b << 0, 0, 0, // pos
			1, 0, 0, 0; // identity quaternion
	return q_b;
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
