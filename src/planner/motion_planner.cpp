#include "planner/motion_planner.hpp"

MotionPlanner::MotionPlanner()
{
	InitRos();
	InitCmdVariables();
	SetRobotMode(kIdle);

	vel_cmd_ = Eigen::Vector2d(0.25, 0); // TODO: take from ros topic
	gait_sequence_ = CreateSimpleSequence();
	//gait_sequence_ = CreateCrawlSequence();
}

bool MotionPlanner::IsReady() // TODO: remove this?
{
	return received_first_state_msg_;
}

void MotionPlanner::Update(const double time)
{
	double seconds_since_start = GetElapsedTimeSince(start_time_);

	switch(robot_mode_)
	{
		case kIdle:
			// Do not publish anything
			break;	
		case kStandup:
			UpdateStandupCmd(seconds_since_start);
			PublishMotionCmd();
			break;	
		case kWalk:
			UpdateWalkCmd(seconds_since_start);
			PublishMotionCmd();
			PublishVisualization(seconds_since_start);
			break;
	}
}

// TODO: For now, this assumes that we always start at the first gait step
void MotionPlanner::GenerateWalkCmdTraj()
{
	curr_2d_pos_ = q_.block<k2D,1>(kQuatSize,0);
	curr_height_ = q_(kQuatSize + 2);

	assert(received_first_state_msg_ == true);
	robot_dynamics_.SetState(q_,u_);

	GenerateWalkLegsPlan();
	GenerateWalkBasePlan();
}

void MotionPlanner::UpdateStandupCmd(const double time)
{
	base_pos_cmd_ = base_planner_.EvalStandupPosTrajAtT(time);
	base_vel_cmd_ = base_planner_.EvalStandupVelTrajAtT(time);
	base_acc_cmd_ = base_planner_.EvalStandupAccTrajAtT(time);
	contact_pattern_cmd_ = leg_planner_.GetAllLegsContact();
}

void MotionPlanner::UpdateWalkCmd(const double time)
{
	// TODO: there is a mistake here where base trajectory is published as 2D and not 3D!
	UpdateWalkBaseCmd(time);
	UpdateWalkLegCmd(time);
}

void MotionPlanner::UpdateWalkBaseCmd(const double time)
{
	base_pos_cmd_ = base_planner_.EvalWalkTrajPosAtT(time);
	base_vel_cmd_ = base_planner_.EvalWalkTrajVelAtT(time);
	base_acc_cmd_ = base_planner_.EvalWalkTrajAccAtT(time);
}

void MotionPlanner::UpdateWalkLegCmd(const double time)
{
	legs_pos_cmd_ = leg_planner_.GetStackedLegPosAtT(time);
	legs_vel_cmd_ = leg_planner_.GetStackedLegVelAtT(time);
	legs_acc_cmd_ = leg_planner_.GetStackedLegAccAtT(time);
	contact_pattern_cmd_ = leg_planner_.GetContactPatternAtT(time);
}

// ****************** //
// COMMAND PUBLISHING //
// ****************** //

void MotionPlanner::PublishMotionCmd()
{
	PublishBaseTrajectories();
	PublishLegTrajectories();
}

void MotionPlanner::PublishBaseTrajectories()
{
	PublishBasePosCmd();
	PublishBaseVelCmd();
	PublishBaseAccCmd();
}

void MotionPlanner::PublishBasePosCmd()
{
	std_msgs::Float64MultiArray base_pos_cmd_msg;
	tf::matrixEigenToMsg(base_pos_cmd_, base_pos_cmd_msg);
	base_pos_traj_pub_.publish(base_pos_cmd_msg);
}

void MotionPlanner::PublishBaseVelCmd()
{
	std_msgs::Float64MultiArray base_vel_cmd_msg;
	tf::matrixEigenToMsg(base_vel_cmd_, base_vel_cmd_msg);
	base_vel_traj_pub_.publish(base_vel_cmd_msg);
}

void MotionPlanner::PublishBaseAccCmd()
{
	std_msgs::Float64MultiArray base_acc_cmd_msg;
	tf::matrixEigenToMsg(base_acc_cmd_, base_acc_cmd_msg);
	base_acc_traj_pub_.publish(base_acc_cmd_msg);
}

void MotionPlanner::PublishLegTrajectories()
{
	PublishLegPosCmd();
	PublishLegVelCmd();
	PublishLegAccCmd();
	PublishContactPattern();
}

void MotionPlanner::PublishLegPosCmd()
{
	std_msgs::Float64MultiArray legs_pos_cmd_msg;
	tf::matrixEigenToMsg(legs_pos_cmd_, legs_pos_cmd_msg);
	legs_pos_cmd_pub_.publish(legs_pos_cmd_msg);
}

void MotionPlanner::PublishLegVelCmd()
{
	std_msgs::Float64MultiArray legs_vel_cmd_msg;
	tf::matrixEigenToMsg(legs_vel_cmd_, legs_vel_cmd_msg);
	legs_vel_cmd_pub_.publish(legs_vel_cmd_msg);
}

void MotionPlanner::PublishLegAccCmd()
{
	std_msgs::Float64MultiArray legs_acc_cmd_msg;
	tf::matrixEigenToMsg(legs_acc_cmd_, legs_acc_cmd_msg);
	legs_acc_cmd_pub_.publish(legs_acc_cmd_msg);
}

void MotionPlanner::PublishContactPattern()
{
	std_msgs::Float64MultiArray contact_pattern_msg;
	tf::matrixEigenToMsg(contact_pattern_cmd_, contact_pattern_msg);
	contact_pattern_pub_.publish(contact_pattern_msg);
}

// ************* //
// VISUALIZATION //
// ************* //

void MotionPlanner::PublishVisualization(const double time)
{
	PublishLegTrajectoriesVisualization();
	PublishBaseTrajVisualization();
	PublishPolygonVisualizationAtTime(time);
}

// TODO: These visualization functions are uneccesarily messy
// TODO: move these to their own module!
void MotionPlanner::PublishPolygonVisualizationAtTime(const double time)
{
	const std::vector<Eigen::Vector2d> polygon_at_t =
		leg_planner_.GetSupportPolygonAtT(time);

	visualization_msgs::Marker polygon_msg;
	polygon_msg.header.frame_id = "world";
	polygon_msg.header.stamp = ros::Time::now();
	polygon_msg.ns = "polygons";
	polygon_msg.action = visualization_msgs::Marker::ADD;
	polygon_msg.pose.orientation.w = 1.0; // Set no rotation
	polygon_msg.type = visualization_msgs::Marker::LINE_STRIP;

	polygon_msg.scale.x = 0.01; // = width
	polygon_msg.color.b = 1.0;
	polygon_msg.color.a = 1.0;

	geometry_msgs::Point p;

	polygon_msg.points.clear(); // clear previous points
	polygon_msg.id = 0; // Replace the polygon on every iteration

	for (
			int point_j = 0;
			point_j < polygon_at_t.size();
			++point_j
			)
	{
		p.x = polygon_at_t[point_j](0);
		p.y = polygon_at_t[point_j](1);
		p.z = 0;
		polygon_msg.points.push_back(p);
	}

	// Close polygon
	p.x = polygon_at_t[0](0);
	p.y = polygon_at_t[0](1);
	p.z = 0;
	polygon_msg.points.push_back(p);

	visualize_polygons_pub_.publish(polygon_msg);
}

void MotionPlanner::PublishBaseTrajVisualization()
{
	visualization_msgs::Marker
		traj_points, start_point, line_strip;
	line_strip.header.frame_id
		= start_point.header.frame_id
		= traj_points.header.frame_id
		= "world";
	line_strip.header.stamp
		= start_point.header.stamp
		= traj_points.header.stamp
		= ros::Time::now();
	line_strip.ns
		= start_point.ns
		= traj_points.ns
		= "trajectory";
	line_strip.action
		= start_point.action
		= traj_points.action
		= visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w
		= start_point.pose.orientation.w
		= traj_points.pose.orientation.w
		= 1.0; // Set no rotation: The rest set to 0 by initialization
	line_strip.id = 0;
	start_point.id = 1;
	traj_points.id = 2;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	start_point.type = visualization_msgs::Marker::POINTS;
	traj_points.type = visualization_msgs::Marker::POINTS;

	// Configure figure
	start_point.scale.x = 0.05;
  start_point.scale.y = 0.05;
	start_point.color.r = 1.0;
	start_point.color.a = 1.0;

  traj_points.scale.x = 0.02;
  traj_points.scale.y = 0.02;
	traj_points.color.g = 1.0;
	traj_points.color.a = 1.0;

	line_strip.scale.x = 0.01; // = width
	line_strip.color.g = 1.0;
	line_strip.color.a = 1.0;

	// Publish initial and final point
	geometry_msgs::Point p;
	p.x = curr_2d_pos_(0);
	p.y = curr_2d_pos_(1);
	start_point.points.push_back(p);

	const int n_traj_segments = base_planner_.GetNumTrajSegments();

	for (int k = 1; k < n_traj_segments; ++k)
	{
		Eigen::VectorXd p_xy = base_planner_.EvalWalkTrajPosAtT(k);
		p.x = p_xy(0);
		p.y = p_xy(1);
		p.z = 0;

		traj_points.points.push_back(p);
	}

	for (
			double t = 0;
			t < n_traj_segments - visualization_resolution_;
			t += visualization_resolution_
			)
	{
		Eigen::VectorXd p_xy = base_planner_.EvalWalkTrajPosAtT(t);

		p.x = p_xy(0);
		p.y = p_xy(1);
		p.z = 0;

		line_strip.points.push_back(p);
	}

	visualize_base_traj_pub_.publish(line_strip);
	visualize_base_traj_pub_.publish(traj_points);
	visualize_base_traj_pub_.publish(start_point);
}

void MotionPlanner::PublishLegTrajectoriesVisualization()
{
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		visualization_msgs::Marker line_strip;
		line_strip.header.frame_id = "world";
		line_strip.header.stamp = ros::Time::now();
		line_strip.ns = "leg_trajectories";
		line_strip.action = visualization_msgs::Marker::ADD;
		// Set no rotation: The rest set to 0 by initialization
		line_strip.pose.orientation.w = 1.0;

		line_strip.id = leg_i;
		line_strip.type = visualization_msgs::Marker::LINE_STRIP;

		line_strip.scale.x = 0.01; // = width
		line_strip.color.g = 1.0;
		line_strip.color.a = 1.0;

		geometry_msgs::Point p;
		const double dt = 0.1;
		for (double t = leg_planner_.GetLegTrajStartTime(leg_i);
				t <= leg_planner_.GetLegTrajEndTime(leg_i); t += visualization_resolution_)
		{
			Eigen::VectorXd pos_at_t = leg_planner_.GetLegPosAtT(t, leg_i);

			p.x = pos_at_t(0);
			p.y = pos_at_t(1);
			p.z = pos_at_t(2);

			line_strip.points.push_back(p);
		}

		visualize_leg_traj_pub_.publish(line_strip);
	}
}

// *** //
// ROS //
// *** //

void MotionPlanner::InitRos()
{
	SetupVisualizationTopics();
	SetupBaseCmdTopics();
	SetupLegCmdTopics();
	SetupStateSubscription();
	SetupServices();
}

void MotionPlanner::SetupServices()
{
	ros::AdvertiseServiceOptions cmd_standup_aso =
		ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
					"/" + model_name_ + "/standup",
					boost::bind(&MotionPlanner::CmdStandupService, this, _1, _2),
					ros::VoidPtr(), NULL
			);
	cmd_standup_service_ = ros_node_.advertiseService(cmd_standup_aso);

	ros::AdvertiseServiceOptions cmd_walk_aso =
		ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
					"/" + model_name_ + "/walk",
					boost::bind(&MotionPlanner::CmdWalkService, this, _1, _2),
					ros::VoidPtr(), NULL
			);
	cmd_walk_service_ = ros_node_.advertiseService(cmd_walk_aso);
}

bool MotionPlanner::CmdStandupService(
				const std_srvs::Empty::Request &_req,
				std_srvs::Empty::Response &_res
		)
{
	SetRobotMode(kStandup);
	return true;
}

bool MotionPlanner::CmdWalkService(
				const std_srvs::Empty::Request &_req,
				std_srvs::Empty::Response &_res
		)
{
	SetRobotMode(kWalk);
	return true;
}

void MotionPlanner::SetupBaseCmdTopics()
{
  base_pos_traj_pub_ =
		ros_node_.advertise<std_msgs::Float64MultiArray>
		("base_pos_cmd", 10);

  base_vel_traj_pub_ =
		ros_node_.advertise<std_msgs::Float64MultiArray>
		("base_vel_cmd", 10);

  base_acc_traj_pub_ =
		ros_node_.advertise<std_msgs::Float64MultiArray>
		("base_acc_cmd", 10);
}

void MotionPlanner::SetupVisualizationTopics()
{
  visualize_base_traj_pub_ =
		ros_node_.advertise<visualization_msgs::Marker>
		("visualization_trajectory", 10);

  visualize_leg_traj_pub_ =
		ros_node_.advertise<visualization_msgs::Marker>
		("leg_visualization_trajectory", 10);

  visualize_polygons_pub_ =
		ros_node_.advertise<visualization_msgs::Marker>
		("visualization_polygons", 10);
}

void MotionPlanner::SetupLegCmdTopics()
{
  contact_pattern_pub_ =
		ros_node_.advertise<std_msgs::Float64MultiArray>
		("legs_contact_cmd", 10);

  legs_pos_cmd_pub_ =
		ros_node_.advertise<std_msgs::Float64MultiArray>
		("legs_pos_cmd", 10);

  legs_vel_cmd_pub_ =
		ros_node_.advertise<std_msgs::Float64MultiArray>
		("legs_vel_cmd", 10);

  legs_acc_cmd_pub_ =
		ros_node_.advertise<std_msgs::Float64MultiArray>
		("legs_acc_cmd", 10);
}

void MotionPlanner::SetupStateSubscription()
{
	gen_coord_sub_ = ros_node_
		.subscribe<std_msgs::Float64MultiArray>(
			"/" + model_name_ + "/gen_coord",
			1,
			boost::bind(&MotionPlanner::OnGenCoordMsg, this, _1)
			);

	gen_vel_sub_ = ros_node_
		.subscribe<std_msgs::Float64MultiArray>(
			"/" + model_name_ + "/gen_vel",
			1,
			boost::bind(&MotionPlanner::OnGenVelMsg, this, _1)
			);
}

void MotionPlanner::OnGenCoordMsg(
		const std_msgs::Float64MultiArrayConstPtr &msg
		)
{
	received_first_state_msg_ = true;
	SetGenCoords(msg->data);
}

void MotionPlanner::OnGenVelMsg(
		const std_msgs::Float64MultiArrayConstPtr &msg
		)
{
	received_first_state_msg_ = true;
	SetGenVels(msg->data);
}

void MotionPlanner::SetGenCoords(const std::vector<double> &gen_coords)
{
	for (int i = 0; i < kNumGenCoords; ++i)
		q_(i) = gen_coords[i];
}

void MotionPlanner::SetGenVels(const std::vector<double> &gen_vels)
{
	for (int i = 0; i < 18; ++i)
		u_(i) = gen_vels[i];
}

// ************* //
// STATE MACHINE //
// ************* //

void MotionPlanner::SetRobotMode(RobotMode target_mode)
{
	switch(target_mode)
	{
		case kIdle:
			ROS_INFO("Setting mode to IDLE");
			break;
		case kStandup:
			{
				ROS_INFO("Setting mode to STANDUP");
				InitCmdVariables();
				const Eigen::VectorXd curr_pos =
					q_.block<k3D, 1>(kQuatSize, 0);
				const double target_height = 0.2;
				const double time_to_standup = 2.0;
				base_planner_.PlanBaseStandupMotion(
						time_to_standup, target_height, curr_pos
						);
				break;
			}
		case kWalk:
			ROS_INFO("Setting mode to WALK");
			GenerateWalkCmdTraj();
			break;
		default:
			break;
	}

	robot_mode_ = target_mode;
	start_time_ = ros::Time::now();
}

// ******************* //
// GAIT AND TRAJECTORY //
// ******************* //

void MotionPlanner::InitCmdVariables()
{
	base_pos_cmd_.resize(k3D);
	base_pos_cmd_.setZero();
	base_vel_cmd_.resize(k3D);
	base_vel_cmd_.setZero();
	base_acc_cmd_.resize(k3D);
	base_acc_cmd_.setZero();

	legs_pos_cmd_.resize(k3D * kNumLegs);
	legs_pos_cmd_.setZero();
	legs_vel_cmd_.resize(k3D * kNumLegs);
	legs_vel_cmd_.setZero();
	legs_acc_cmd_.resize(k3D * kNumLegs);
	contact_pattern_cmd_.resize(kNumLegs);
	contact_pattern_cmd_.setZero();
}
 
GaitSequence MotionPlanner::CreateSimpleSequence()
{
	int n_gait_steps = 12;
	double gait_duration = 20;
	double gait_step_time = gait_duration / (double) n_gait_steps;

	Eigen::MatrixXd gait_contact_schedule(kNumLegs, n_gait_steps);
	gait_contact_schedule << 
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1,
		1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
		1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1,
		1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;


	GaitSequence simple_gait = {
		n_gait_steps, gait_duration, gait_step_time, gait_contact_schedule 
	};

	return simple_gait;
}

GaitSequence MotionPlanner::CreateCrawlSequence()
{
	int n_gait_steps = 20;
	double gait_duration = 10;
	double gait_step_time = gait_duration / (double) n_gait_steps;

	Eigen::MatrixXd gait_contact_schedule(kNumLegs, n_gait_steps);
	gait_contact_schedule << 
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1,
		1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1,
		1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;

	GaitSequence crawl_gait = {
		n_gait_steps, gait_duration, gait_step_time, gait_contact_schedule 
	};

	return crawl_gait;
}

void MotionPlanner::GenerateWalkLegsPlan()
{
	// TODO: remove
	const std::vector<int> all_feet = {0,1,2,3};
	Eigen::MatrixXd current_stance =
		robot_dynamics_.GetStacked2DFootPosInW(all_feet);

	leg_planner_.SetGaitSequence(gait_sequence_);
	leg_planner_.PlanLegsMotion(vel_cmd_, current_stance);
}

void MotionPlanner::GenerateWalkBasePlan()
{
	const int polynomial_degree = 5;
	const int n_traj_segments = 10; // TODO: move

	base_planner_.SetCurrPos(curr_2d_pos_);
	base_planner_.SetSupportPolygons(leg_planner_.GetSupportPolygons());
	base_planner_.PlanBaseWalkMotion(
			polynomial_degree, n_traj_segments, curr_height_
			);
}

// **************** //
// HELPER FUNCTIONS //
// **************** //

double MotionPlanner::GetElapsedTimeSince(ros::Time t)
{
	double elapsed_time = (ros::Time::now() - t).toSec();
	return elapsed_time; 
}

// ******** //
// ROS NODE //
// ******** //

int main( int argc, char** argv )
{
  ros::init(argc, argv, "motion_planner");
	MotionPlanner motion_planner;
	ROS_INFO("Running motion planner");

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
		ros::spinOnce(); // fetch messages

		if (!motion_planner.IsReady()) continue;
		
		const double time = ros::Time::now().toSec();

		motion_planner.Update(time);
		ros::spinOnce(); // publish messages
    loop_rate.sleep();
  }
}
