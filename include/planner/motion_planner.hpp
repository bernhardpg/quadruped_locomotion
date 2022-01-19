#pragma once

#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <iomanip>

#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>

#include "dynamics/dynamics.hpp"
#include "planner/leg_planner.hpp"
#include "planner/base_planner.hpp"
#include "planner/gait_sequence.hpp"
#include "helper_functions.hpp"
#include "anymal_constants.hpp"

class MotionPlanner
{
	public:
		MotionPlanner();

		void GenerateTrajectory();

		// ************* //
		// VISUALIZATION //
		// ************* //

		void PublishVisualization(const double time);
		void PublishBaseTrajVisualization();
		void PublishPolygonVisualizationAtTime(const double time);
		void PublishLegTrajectoriesVisualization();

		void PublishBaseTrajectories(const double time);
		void PublishBasePosCmd(const double time);
		void PublishBaseVelCmd(const double time);
		void PublishBaseAccCmd(const double time);

		void PublishLegTrajectories(const double time);
		void PublishLegsInContact(const double time);
		void PublishLegPosCmd(const double time);
		void PublishLegVelCmd(const double time);
		void PublishLegAccCmd(const double time);

	private:
		// *** //
		// ROS //
		// *** //

		ros::NodeHandle ros_node_;
		ros::Publisher visualize_base_traj_pub_; 
		ros::Publisher visualize_leg_traj_pub_; 
		ros::Publisher visualize_polygons_pub_; 

		ros::Publisher legs_in_contact_pub_; 
		ros::Publisher legs_pos_cmd_pub_; 
		ros::Publisher legs_vel_cmd_pub_; 
		ros::Publisher legs_acc_cmd_pub_; 

		ros::Publisher base_pos_traj_pub_; 
		ros::Publisher base_vel_traj_pub_; 
		ros::Publisher base_acc_traj_pub_; 

		void InitRos();
		void SetupVisualizationTopics();
		void SetupBaseCmdTopics();
		void SetupLegCmdTopics();

		// ******************* //
		// GAIT AND TRAJECTORY //
		// ******************* //

		Eigen::Vector2d pos_initial_;
		Eigen::Vector2d pos_final_;

		LegPlanner leg_planner_;
		BasePlanner base_planner_;
		Dynamics robot_dynamics_;

		GaitSequence gait_sequence_;

		Eigen::Vector2d vel_cmd_;

		double visualization_resolution_ = 0.1;

		GaitSequence CreateCrawlSequence();
};

