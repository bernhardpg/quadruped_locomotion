#include <ros/ros.h>
#include <ros/console.h>
#include "control/controller.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller",
			ros::init_options::NoSigintHandler
			);
	ROS_INFO("Starting controller node");
	int frequency = 400;
	control::Controller controller(frequency);
	ros::spin();

	return 0;
}
