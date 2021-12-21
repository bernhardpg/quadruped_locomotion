#include "planner/motion_planner.hpp"


#include <cmath>

int main( int argc, char** argv )
{

	int traj_degree = 5;
	int n_traj_segments = 10;

  ros::init(argc, argv, "motion_planner_node");
	MotionPlanner planner(traj_degree, n_traj_segments);
	planner.GenerateTrajectory();

  ros::Rate r(30);
  while (ros::ok())
  {
		planner.PublishTrajectoryVisualization();
		planner.PublishPolygonsVisualization();
    r.sleep();
  }
}
