#pragma once

#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <iomanip>

#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
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

		bool IsReady();

		void Update(const double time);
		void GenerateWalkCmdTraj();
		void PublishMotionCmd();

		void PublishVisualization(const double time);

	private:

		// *** //
		// ROS //
		// *** //

		ros::Time start_time_;

		ros::NodeHandle ros_node_;
		const std::string model_name_ = "anymal";

		ros::Publisher visualize_base_traj_pub_; 
		ros::Publisher visualize_leg_traj_pub_; 
		ros::Publisher visualize_polygons_pub_; 

		ros::Publisher contact_pattern_pub_; 
		ros::Publisher legs_pos_cmd_pub_; 
		ros::Publisher legs_vel_cmd_pub_; 
		ros::Publisher legs_acc_cmd_pub_; 

		ros::Publisher base_pos_traj_pub_; 
		ros::Publisher base_vel_traj_pub_; 
		ros::Publisher base_acc_traj_pub_; 

		ros::Subscriber gen_coord_sub_;
		ros::Subscriber gen_vel_sub_;

		ros::ServiceServer cmd_standup_service_;	
		ros::ServiceServer cmd_walk_service_;	

		bool received_first_state_msg_ = false;

		void InitRos();
		void SetupServices();
		void SetupVisualizationTopics();
		void SetupBaseCmdTopics();
		void SetupLegCmdTopics();
		void SetupStateSubscription();

		bool CmdStandupService(
				const std_srvs::Empty::Request &_req,
				std_srvs::Empty::Response &_res
				);
		bool CmdWalkService(
				const std_srvs::Empty::Request &_req,
				std_srvs::Empty::Response &_res
				);


		void OnGenCoordMsg(const std_msgs::Float64MultiArrayConstPtr &msg);
		void OnGenVelMsg(const std_msgs::Float64MultiArrayConstPtr &msg);
		void SetGenCoords(const std::vector<double> &gen_coords);
		void SetGenVels(const std::vector<double> &gen_vels);

		// ************* //
		// STATE MACHINE //
		// ************* //

		enum RobotMode {
			kIdle, kStandup, kWalk
		} robot_mode_;

		void SetRobotMode(RobotMode target_mode);

		// ******************* //
		// GAIT AND TRAJECTORY //
		// ******************* //

		gen_coord_vector_t q_;
		gen_vel_vector_t u_;

		LegPlanner leg_planner_;
		BasePlanner base_planner_;
		Dynamics robot_dynamics_;

		GaitSequence gait_sequence_;

		Eigen::Vector2d vel_cmd_;
		Eigen::Vector2d curr_2d_pos_;
		double curr_height_;
		
		Eigen::VectorXd legs_pos_cmd_;
		Eigen::VectorXd legs_vel_cmd_;
		Eigen::VectorXd legs_acc_cmd_;
		Eigen::VectorXd contact_pattern_cmd_;

		Eigen::VectorXd base_pos_cmd_;
		Eigen::VectorXd base_vel_cmd_;
		Eigen::VectorXd base_acc_cmd_;

		GaitSequence CreateCrawlSequence();
		GaitSequence CreateSimpleSequence();

		void InitCmdVariables();

		void GenerateWalkLegsPlan();
		void GenerateWalkBasePlan();
		void UpdateStandupCmd(const double time);
		void UpdateWalkCmd(const double time);

		void UpdateWalkBaseCmd(const double time);
		void UpdateWalkLegCmd(const double time);

		// ****************** //
		// COMMAND PUBLISHING //
		// ****************** //

		void PublishBaseTrajectories();
		void PublishBasePosCmd();
		void PublishBaseVelCmd();
		void PublishBaseAccCmd();

		void PublishLegTrajectories();
		void PublishContactPattern();
		void PublishLegPosCmd();
		void PublishLegVelCmd();
		void PublishLegAccCmd();

		// **************** //
		// HELPER FUNCTIONS //
		// **************** //

		double GetElapsedTimeSince(ros::Time t);

		// ************* //
		// VISUALIZATION //
		// ************* //

		// TODO: This should probably be moved to its own module

		double visualization_resolution_ = 0.1;

		void PublishBaseTrajVisualization(const double curr_time);
		void PublishPolygonVisualizationAtTime(const double time);
		void PublishLegTrajectoriesVisualization();
};

