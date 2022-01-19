#include "planner/motion_planner.hpp"

MotionPlanner::MotionPlanner()
{
	// TODO: I will need to also initialize the start time
	InitRos();

	vel_cmd_ = Eigen::Vector2d(0.25, 0); // TODO: take from ros topic
	gait_sequence_ = CreateCrawlSequence();
}

bool MotionPlanner::IsReady()
{
	return received_first_state_msg_;
}

// TODO: For now, this assumes that we always start at the first gait step
void MotionPlanner::GenerateTrajectory()
{
	curr_2d_pos_ = q_.block<k2D,1>(kQuatSize,0);

	assert(received_first_state_msg_ == true);
	robot_dynamics_.SetState(q_,u_);

	GenerateLegsPlan();
	GenerateBasePlan();
	start_time_ = ros::Time::now();
}

void MotionPlanner::GenerateLegsPlan()
{
	Eigen::MatrixXd current_stance =
		robot_dynamics_.GetStacked2DFootPosInW();

	leg_planner_.SetGaitSequence(gait_sequence_);
	leg_planner_.PlanLegsMotion(vel_cmd_, current_stance);
}

void MotionPlanner::GenerateBasePlan()
{
	const int polynomial_degree = 5;
	const int n_traj_segments = 10; // TODO: move

	base_planner_.SetCurrPos(curr_2d_pos_);
	base_planner_.SetSupportPolygons(leg_planner_.GetSupportPolygons());
	base_planner_.PlanBaseMotion(polynomial_degree, n_traj_segments);
}

void MotionPlanner::PublishMotionCmd(const double time)
{
	double seconds_since_start = GetElapsedTimeSince(start_time_);
	PublishBaseTrajectories(seconds_since_start);
	PublishLegTrajectories(seconds_since_start);
}

void MotionPlanner::PublishBaseTrajectories(const double time)
{
	PublishBasePosCmd(time);
	PublishBaseVelCmd(time);
	PublishBaseAccCmd(time);
}

void MotionPlanner::PublishBasePosCmd(const double time)
{
	std_msgs::Float64MultiArray base_pos_cmd;
	tf::matrixEigenToMsg(
			base_planner_.EvalBasePosAtT(time), base_pos_cmd
			);
	base_pos_traj_pub_.publish(base_pos_cmd);
}

void MotionPlanner::PublishBaseVelCmd(const double time)
{
	std_msgs::Float64MultiArray base_vel_cmd;
	tf::matrixEigenToMsg(
			base_planner_.EvalBaseVelAtT(time), base_vel_cmd
			);
	base_vel_traj_pub_.publish(base_vel_cmd);
}

void MotionPlanner::PublishBaseAccCmd(const double time)
{
	std_msgs::Float64MultiArray base_acc_cmd;
	tf::matrixEigenToMsg(
			base_planner_.EvalBaseAccAtT(time), base_acc_cmd
			);
	base_acc_traj_pub_.publish(base_acc_cmd);
}

void MotionPlanner::PublishLegTrajectories(const double time)
{
	double time_rel = std::fmod(time, gait_sequence_.duration); // TODO: these should be replaced with a common time

	PublishLegPosCmd(time_rel);
	PublishLegVelCmd(time_rel);
	PublishLegAccCmd(time_rel);
	PublishLegsInContact(time_rel);
}

void MotionPlanner::PublishLegPosCmd(const double time)
{
	std_msgs::Float64MultiArray legs_pos_cmd;
	tf::matrixEigenToMsg(
			leg_planner_.GetStackedLegPosAtT(time), legs_pos_cmd
			);
	legs_pos_cmd_pub_.publish(legs_pos_cmd);
}

void MotionPlanner::PublishLegVelCmd(const double time)
{
	std_msgs::Float64MultiArray legs_vel_cmd;
	tf::matrixEigenToMsg(
			leg_planner_.GetStackedLegVelAtT(time), legs_vel_cmd
			);
	legs_vel_cmd_pub_.publish(legs_vel_cmd);
}

void MotionPlanner::PublishLegAccCmd(const double time)
{
	std_msgs::Float64MultiArray legs_acc_cmd;
	tf::matrixEigenToMsg(
			leg_planner_.GetStackedLegAccAtT(time), legs_acc_cmd
			);
	legs_acc_cmd_pub_.publish(legs_acc_cmd);
}

void MotionPlanner::PublishLegsInContact(const double time)
{
	std_msgs::Float64MultiArray legs_in_contact_msg;
	tf::matrixEigenToMsg(leg_planner_.GetLegsInContactAtT(time), legs_in_contact_msg);
	legs_in_contact_pub_.publish(legs_in_contact_msg);
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
		Eigen::VectorXd p_xy = base_planner_.EvalBasePosAtT(k);
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
		Eigen::VectorXd p_xy = base_planner_.EvalBasePosAtT(t);

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
  legs_in_contact_pub_ =
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

// ******************* //
// GAIT AND TRAJECTORY //
// ******************* //

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

// **************** //
// HELPER FUNCTIONS //
// **************** //

double MotionPlanner::GetElapsedTimeSince(ros::Time t)
{
	double elapsed_time = (ros::Time::now() - t).toSec();
	return elapsed_time; 
}

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
		ROS_INFO("Generating new motion plan");
		
		const double time = ros::Time::now().toSec();
		motion_planner.GenerateTrajectory();
		motion_planner.PublishMotionCmd(time);
		motion_planner.PublishVisualization(time);

		ros::spinOnce(); // publish messages
    loop_rate.sleep();
  }
}
