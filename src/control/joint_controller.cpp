#include "control/joint_controller.hpp"

namespace control
{
	JointController::JointController()
		: JointController(2500)
	{}

	JointController::JointController(int frequency_)
		: loop_rate_(frequency_)
	{
		ROS_INFO("Starting joint controller at %d Hz", frequency_);
		model_name_ = "anymal"; // TODO: Take model name as an argument to the node

		q_j_.setZero();
		q_j_dot_.setZero();
		q_j_cmd_.setZero();
		q_j_dot_cmd_.setZero();
		tau_cmd_.setZero();

		InitDynamicReconfigureRos();
		InitRos();
	}
		
	void JointController::InitRos()
	{
		torque_cmd_pub_ = node_handle_
			.advertise<std_msgs::Float64MultiArray>(
					"/" + model_name_ + "/torque_cmd", 1
					);

		q_joint_cmd_sub_ = node_handle_
			.subscribe<std_msgs::Float64MultiArray>(
					"/q_j_cmd", 1,
					boost::bind(&JointController::OnJointPosCmdMsg, this, _1)
					);

		q_joint_dot_cmd_sub_ = node_handle_
			.subscribe<std_msgs::Float64MultiArray>(
					"/q_j_dot_cmd", 1,
					boost::bind(&JointController::OnJointVelCmdMsg, this, _1)
					);

		gen_coord_sub_ = node_handle_
			.subscribe<std_msgs::Float64MultiArray>(
					"/gen_coord", 1,
					boost::bind(&JointController::OnGenCoordMsg, this, _1)
					);

		gen_vel_sub_ = node_handle_
			.subscribe<std_msgs::Float64MultiArray>(
					"/gen_vel", 1,
					boost::bind(&JointController::OnGenVelMsg, this, _1)
					);

		// Single threaded
		while(ros::ok())
		{
			ros::spinOnce(); // spin once to fetch newest messages
			CalcJointTorques();

			std_msgs::Float64MultiArray torque_cmd_msg;
			tau_cmd_.block<9,1>(0,0).setZero();
			tf::matrixEigenToMsg(tau_cmd_, torque_cmd_msg);

			torque_cmd_pub_.publish(torque_cmd_msg);

			ros::spinOnce(); // publish messages
			loop_rate_.sleep();
		}
	}

	void JointController::InitDynamicReconfigureRos()
	{
		dc_callback_ = boost::bind(
				&JointController::DynamicConfigureCallback, this, _1, _2
				);
		dc_server_.setCallback(dc_callback_);
	}


	void JointController::DynamicConfigureCallback(
			quadruped_locomotion::DynamicControlConfig &config,
			uint32_t level
			)
	{
		ROS_INFO("Reconfigure Request: %f %f",
								config.k_joints_p, config.k_joints_d);

		k_joints_p_ = config.k_joints_p;
		k_joints_d_ = config.k_joints_d;
	} 

	void JointController::CalcJointTorques()
	{
		tau_cmd_ = k_joints_p_ * (q_j_cmd_ - q_j_)
			+ k_joints_d_ * (q_j_dot_cmd_ - q_j_dot_);
					
	}

	void JointController::OnGenCoordMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		for (size_t i = kNumGenCoords - kNumJoints; i < kNumGenCoords; ++i)
		{
			q_j_(i)	 = msg->data[i]; // Only store joint positions
		}
	}

	void JointController::OnGenVelMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		for (size_t i = kNumGenVels - kNumJoints; i < kNumGenVels; ++i)
		{
			q_j_dot_(i)	 = msg->data[i]; // Only store joint positions
		}
	}

	void JointController::OnJointPosCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		for (size_t i = 0; i < kNumJoints; ++i)
		{
			q_j_cmd_(i)	 = msg->data[i];
		}
	}

	void JointController::OnJointVelCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		for (size_t i = 0; i < kNumJoints; ++i)
		{
			q_j_dot_cmd_(i)	 = msg->data[i];
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_controller",
		ros::init_options::NoSigintHandler
		);
	int frequency = 2500;
	control::JointController joint_controller_(2500);
	return 0;
}
