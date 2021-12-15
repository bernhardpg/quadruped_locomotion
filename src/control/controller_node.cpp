#include "ros/ros.h"
#include "control/controller.hpp"

int main(int argc, char **argv)
{
	control::Controller controller;
	ros::spin();

	return 0;
}
