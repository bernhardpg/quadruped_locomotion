#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "motion_planner_node");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "world";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
		line_strip.ns = "trajectory";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0; // Set no rotation: The rest set to 0 by initialization

    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

		for (int i = 0; i < 10; ++i)
		{
			geometry_msgs::Point p;
			p.x = i;
			p.y = i;
			p.z = 0;

			points.points.push_back(p);
			line_strip.points.push_back(p);
		}

    marker_pub.publish(line_strip);

    r.sleep();
  }
}
