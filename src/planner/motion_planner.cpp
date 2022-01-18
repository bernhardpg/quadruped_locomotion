#include "planner/motion_planner.hpp"

// TODO: clean up this class

MotionPlanner::MotionPlanner(
		int degree,
		int n_traj_segments
		)
	:
		n_traj_segments_(n_traj_segments),
		degree_(degree)
{
	// TODO: I will need to also initialize the start time
	InitRos();

	vel_cmd_ = Eigen::Vector2d(0.25, 0); // TODO: take from ros topic
	InitGaitSequence();

	// TODO: Remember to update robot dynamcis with gen coords
	robot_dynamics_.SetStateDefault();
	current_stance_ = robot_dynamics_.GetStacked2DFootPosInW();
	// TODO: For now, this assumes that we always start at the first gait step
	stance_sequence_ = GenerateStanceSequence(vel_cmd_, current_stance_);

	GenerateSupportPolygons();
	SetupOptimizationProgram(); // TODO: move this to the right spot

	leg_motions_ = CreateLegMotions();
	leg_trajectories_ = CreateLegTrajectories(leg_motions_);
}

void MotionPlanner::PublishComTrajectories(const double time)
{
	PublishComPosCmd(time);
	PublishComVelCmd(time);
	PublishComAccCmd(time);
}

void MotionPlanner::PublishComPosCmd(const double time)
{
	std_msgs::Float64MultiArray com_pos_cmd;
	tf::matrixEigenToMsg(
			EvalComPosAtT(time), com_pos_cmd // TODO: add vel and acceleration
			);
	com_pos_traj_pub_.publish(com_pos_cmd);
}

void MotionPlanner::PublishComVelCmd(const double time)
{
	std_msgs::Float64MultiArray com_vel_cmd;
	tf::matrixEigenToMsg(
			EvalComVelAtT(time), com_vel_cmd // TODO: add vel and acceleration
			);
	com_vel_traj_pub_.publish(com_vel_cmd);
}

void MotionPlanner::PublishComAccCmd(const double time)
{
	std_msgs::Float64MultiArray com_acc_cmd;
	tf::matrixEigenToMsg(
			EvalComAccAtT(time), com_acc_cmd // TODO: add vel and acceleration
			);
	com_acc_traj_pub_.publish(com_acc_cmd);
}

void MotionPlanner::PublishLegTrajectories(const double time)
{
	PublishLegPosCmd(time);
	PublishLegVelCmd(time);
	PublishLegAccCmd(time);
	PublishLegsInContact(time);
}

void MotionPlanner::PublishLegPosCmd(const double time)
{
	std_msgs::Float64MultiArray legs_pos_cmd;
	tf::matrixEigenToMsg(
			GetStackedLegPosAtT(time), legs_pos_cmd
			);
	legs_pos_cmd_pub_.publish(legs_pos_cmd);
}

void MotionPlanner::PublishLegVelCmd(const double time)
{
	std_msgs::Float64MultiArray legs_vel_cmd;
	tf::matrixEigenToMsg(
			GetStackedLegVelAtT(time), legs_vel_cmd
			);
	legs_vel_cmd_pub_.publish(legs_vel_cmd);
}

void MotionPlanner::PublishLegAccCmd(const double time)
{
	std_msgs::Float64MultiArray legs_acc_cmd;
	tf::matrixEigenToMsg(
			GetStackedLegAccAtT(time), legs_acc_cmd
			);
	legs_acc_cmd_pub_.publish(legs_acc_cmd);
}

void MotionPlanner::PublishLegsInContact(const double time)
{
	std_msgs::Float64MultiArray legs_in_contact_msg;
	tf::matrixEigenToMsg(GetLegsInContactAtT(time), legs_in_contact_msg);
	legs_in_contact_pub_.publish(legs_in_contact_msg);
}

Eigen::VectorXd MotionPlanner::GetLegsInContactAtT(const double time)
{
	int i = GetGaitStepFromTime(time);
	return gait_sequence_.col(i);
}

Eigen::VectorXd MotionPlanner::GetStackedLegPosAtT(const double time)
{
	Eigen::VectorXd stacked_leg_pos(k3D * kNumLegs);
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		stacked_leg_pos.block<k3D,1>(leg_i * k3D,0) =
			EvalLegPosAtT(time, leg_i);
	}
	return stacked_leg_pos;
}

Eigen::VectorXd MotionPlanner::GetStackedLegVelAtT(const double time)
{
	Eigen::VectorXd stacked_leg_vel(k3D * kNumLegs);
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		stacked_leg_vel.block<k3D,1>(leg_i * k3D,0) =
			EvalLegVelAtT(time, leg_i);
	}
	return stacked_leg_vel;
}

Eigen::VectorXd MotionPlanner::GetStackedLegAccAtT(const double time)
{
	Eigen::VectorXd stacked_leg_acc(k3D * kNumLegs);
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		stacked_leg_acc.block<k3D,1>(leg_i * k3D,0) =
			EvalLegAccAtT(time, leg_i);
	}
	return stacked_leg_acc;
}

Eigen::VectorXd MotionPlanner::EvalLegPosAtT(
		const double time, const int leg_i
		)
{
	const double time_rel = std::fmod(time, t_per_gait_sequence_);
	Eigen::VectorXd leg_pos(3);
	leg_pos.setZero();

	const bool inside_traj_time =
		(time_rel < leg_trajectories_[leg_i].start_time)
			|| (time_rel > leg_trajectories_[leg_i].end_time);

	if (!inside_traj_time)
	{
		leg_pos <<
			leg_trajectories_[leg_i].xy.value(time_rel),
			leg_trajectories_[leg_i].z.value(time_rel);
	}

	return leg_pos;
}

Eigen::VectorXd MotionPlanner::EvalLegVelAtT(
		const double time, const int leg_i
		)
{
	const double time_rel = std::fmod(time, t_per_gait_sequence_);
	Eigen::VectorXd leg_vel(3);
	leg_vel.setZero();

	const bool inside_traj_time =
		(time_rel < leg_trajectories_[leg_i].start_time)
			|| (time_rel > leg_trajectories_[leg_i].end_time);

	if (!inside_traj_time)
	{
		leg_vel <<
			leg_trajectories_[leg_i].xy.value(time_rel),
			leg_trajectories_[leg_i].z.value(time_rel);
	}

	return leg_vel;
}

Eigen::VectorXd MotionPlanner::EvalLegAccAtT(
		const double time, const int leg_i
		)
{
	const double time_rel = std::fmod(time, t_per_gait_sequence_);
	Eigen::VectorXd leg_acc(3);
	leg_acc.setZero();

	const bool inside_traj_time =
		(time_rel < leg_trajectories_[leg_i].start_time)
			|| (time_rel > leg_trajectories_[leg_i].end_time);

	if (!inside_traj_time)
	{
		leg_acc <<
			leg_trajectories_[leg_i].xy.value(time_rel),
			leg_trajectories_[leg_i].z.value(time_rel);
	}

	return leg_acc;
}

// TODO: move all these into a leg motion planner
std::vector<LegTrajectory> MotionPlanner::CreateLegTrajectories(
		std::vector<LegMotion> leg_motions
		)
{
	std::vector<LegTrajectory> leg_trajs;
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		LegTrajectory leg_i_traj; 

		leg_i_traj.start_time = leg_motions[leg_i].t_liftoff;
		leg_i_traj.end_time = leg_motions[leg_i].t_touchdown;

		leg_i_traj.xy = CreateXYLegTrajectory(leg_motions[leg_i]);
		leg_i_traj.z = CreateZLegTrajectory(leg_motions[leg_i]);

		leg_i_traj.d_xy = leg_i_traj.xy.derivative(1);
		leg_i_traj.d_z = leg_i_traj.z.derivative(1);

		leg_i_traj.dd_xy = leg_i_traj.xy.derivative(2);
		leg_i_traj.dd_z = leg_i_traj.z.derivative(2);

		leg_trajs.push_back(leg_i_traj);
	}

	return leg_trajs;
}

drake::trajectories::PiecewisePolynomial<double>
	MotionPlanner::CreateXYLegTrajectory(LegMotion leg_motion)
{
	const std::vector<double> breaks = {
		leg_motion.t_liftoff, leg_motion.t_touchdown
	};

	std::vector<Eigen::MatrixXd> samples = {
		leg_motion.start_pos, leg_motion.end_pos
	};

	drake::trajectories::PiecewisePolynomial<double> xy_traj =
	 	drake::trajectories::PiecewisePolynomial<double>
				::FirstOrderHold(breaks, samples);

	return xy_traj;
}

drake::trajectories::PiecewisePolynomial<double>
	MotionPlanner::CreateZLegTrajectory(LegMotion leg_motion)
{
	const double t_apex = leg_motion.t_liftoff + std::abs(
			leg_motion.t_touchdown - leg_motion.t_liftoff
			) / 2;
	const std::vector<double> breaks = {
		leg_motion.t_liftoff, t_apex, leg_motion.t_touchdown
	};

	const double z_apex_height = 0.2; // TODO: Move to member variable
	Eigen::MatrixXd apex(1,1);
	apex << z_apex_height;

	const Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(1,1);
	const std::vector<Eigen::MatrixXd> samples = {
		zero, apex, zero
	};

	drake::trajectories::PiecewisePolynomial<double> z_traj =
	 	drake::trajectories::PiecewisePolynomial<double>
				::CubicWithContinuousSecondDerivatives(breaks, samples);

	return z_traj;
}

std::vector<LegMotion> MotionPlanner::CreateLegMotions()
{
	std::vector<LegMotion> leg_motions;
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		leg_motions.push_back(CreateLegMotionForLeg(leg_i));
	}

	return leg_motions;
}

LegMotion MotionPlanner::CreateLegMotionForLeg(const int leg_i)
{
	double t_liftoff = -1;
	double t_touchdown = -1;
	Eigen::VectorXd start_pos(k2D);
	Eigen::VectorXd end_pos(k2D);

	// TODO: generalize to start at any configuration
	int last_leg_state = GetLegStateAtStep(0, leg_i);
	for (int gait_step_i = 1; gait_step_i < n_gait_steps_; ++gait_step_i)
	{
		if (GetLegStateAtStep(gait_step_i, leg_i) != last_leg_state)
		{
			if (last_leg_state == 1)	
			{
				t_liftoff = GetTimeAtGaitStep(gait_step_i);
				last_leg_state = 0;
				start_pos = GetFootPosAtStep(gait_step_i - 1, leg_i);
			}
			else
			{
				t_touchdown = GetTimeAtGaitStep(gait_step_i);
				last_leg_state = 1;
				end_pos = GetFootPosAtStep(gait_step_i - 1, leg_i);
			}
		}
	}

	LegMotion leg_i_traj = {t_liftoff, t_touchdown, start_pos, end_pos};
	return leg_i_traj;
}

double MotionPlanner::GetTimeAtGaitStep(int gait_step_i)
{
	double t_at_step = t_per_step_ * gait_step_i;	
	return t_at_step;
}

Eigen::VectorXd MotionPlanner::GetFootPosAtStep(
		int gait_step_i, int leg_i
		)
{
	return stance_sequence_[gait_step_i].col(leg_i);
}

int MotionPlanner::GetLegStateAtStep(int gait_step_i, int leg_i)
{
	return gait_sequence_(leg_i,gait_step_i);
}

Eigen::VectorXd MotionPlanner::EvalComPosAtT(const double t)
{
	return EvalComTrajAtT(t, 0);
}

Eigen::VectorXd MotionPlanner::EvalComVelAtT(const double t)
{
	return EvalComTrajAtT(t, 1);
}

Eigen::VectorXd MotionPlanner::EvalComAccAtT(const double t)
{
	return EvalComTrajAtT(t, 2);
}

Eigen::VectorXd MotionPlanner::EvalComTrajAtT(
		const double t, const int derivative
		)
{
	double t_rel = std::fmod(t, t_per_gait_sequence_); // TODO: these should be replaced with a common time

	int traj_segment_index = 0;
	while ((double) traj_segment_index + 1 < t_rel) ++traj_segment_index;

	double t_in_segment = t_rel - (double) traj_segment_index;

	Eigen::VectorXd traj_value(traj_dimension_);
	drake::symbolic::Environment t_at_t {{t_, t_in_segment}};
	for (int dim = 0; dim < traj_dimension_; ++dim)
	{
		switch(derivative)
		{
			case 0:
				traj_value(dim) = 
					polynomials_pos_(dim, traj_segment_index).Evaluate(t_at_t);
				break;
			case 1:
				traj_value(dim) = 
					polynomials_vel_(dim, traj_segment_index).Evaluate(t_at_t);
				break;
			case 2:
				traj_value(dim) = 
					polynomials_acc_(dim, traj_segment_index).Evaluate(t_at_t);
				break;
		}
	}

	return traj_value;
}

void MotionPlanner::GenerateTrajectory()
{
	result_ = Solve(prog_);
	ROS_INFO_STREAM("Solver id: " << result_.get_solver_id()
		<< "\nFound solution: " << result_.is_success()
		<< "\nSolution result: " << result_.get_solution_result()
		<< std::endl);
	assert(result_.is_success());
	GeneratePolynomialsFromSolution();
}

// ************* //
// VISUALIZATION //
// ************* //

// TODO: These visualization functions are uneccesarily messy
// TODO: move these to their own module!
void MotionPlanner::PublishPolygonVisualizationAtTime(const double time)
{
	int polygon_i = GetGaitStepFromTime(time);

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
			point_j < support_polygons_[polygon_i].size();
			++point_j
			)
	{
		p.x = support_polygons_[polygon_i][point_j](0);
		p.y = support_polygons_[polygon_i][point_j](1);
		p.z = 0;
		polygon_msg.points.push_back(p);
	}

	// Close polygon
	p.x = support_polygons_[polygon_i][0](0);
	p.y = support_polygons_[polygon_i][0](1);
	p.z = 0;
	polygon_msg.points.push_back(p);

	visualize_polygons_pub_.publish(polygon_msg);
}

void MotionPlanner::PublishPolygonsVisualization()
{
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
	for (
			int polygon_i = 0;
			polygon_i < support_polygons_.size();
			++polygon_i
			)
	{
		polygon_msg.points.clear(); // clear previous points
		polygon_msg.id = polygon_i;
		for (
				int point_j = 0;
				point_j < support_polygons_[polygon_i].size();
				++point_j
				)
		{
			p.x = support_polygons_[polygon_i][point_j](0);
			p.y = support_polygons_[polygon_i][point_j](1);
			p.z = 0;
			polygon_msg.points.push_back(p);
		}
		// Close polygon
		p.x = support_polygons_[polygon_i][0](0);
		p.y = support_polygons_[polygon_i][0](1);
		p.z = 0;
		polygon_msg.points.push_back(p);

		visualize_polygons_pub_.publish(polygon_msg);
	}
}

void MotionPlanner::PublishComTrajVisualization()
{
	visualization_msgs::Marker
		traj_points, start_end_points, line_strip;
	line_strip.header.frame_id
		= start_end_points.header.frame_id
		= traj_points.header.frame_id
		= "world";
	line_strip.header.stamp
		= start_end_points.header.stamp
		= traj_points.header.stamp
		= ros::Time::now();
	line_strip.ns
		= start_end_points.ns
		= traj_points.ns
		= "trajectory";
	line_strip.action
		= start_end_points.action
		= traj_points.action
		= visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w
		= start_end_points.pose.orientation.w
		= traj_points.pose.orientation.w
		= 1.0; // Set no rotation: The rest set to 0 by initialization
	line_strip.id = 0;
	start_end_points.id = 1;
	traj_points.id = 2;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	start_end_points.type = visualization_msgs::Marker::POINTS;
	traj_points.type = visualization_msgs::Marker::POINTS;

	// Configure figure
	start_end_points.scale.x = 0.05;
  start_end_points.scale.y = 0.05;
	start_end_points.color.r = 1.0;
	start_end_points.color.a = 1.0;

  traj_points.scale.x = 0.02;
  traj_points.scale.y = 0.02;
	traj_points.color.g = 1.0;
	traj_points.color.a = 1.0;

	line_strip.scale.x = 0.01; // = width
	line_strip.color.g = 1.0;
	line_strip.color.a = 1.0;

	// Publish initial and final point
	geometry_msgs::Point p;
	p.x = pos_initial_(0);
	p.y = pos_initial_(1);
	start_end_points.points.push_back(p);
	p.x = pos_final_(0);
	p.y = pos_final_(1);
	start_end_points.points.push_back(p);

	for (int k = 1; k < n_traj_segments_; ++k)
	{
		Eigen::VectorXd p_xy = EvalComPosAtT(k);
		p.x = p_xy(0);
		p.y = p_xy(1);
		p.z = 0;

		traj_points.points.push_back(p);
	}

	for (double t = 0; t < n_traj_segments_ - dt_; t += dt_)
	{
		Eigen::VectorXd p_xy = EvalComPosAtT(t);

		p.x = p_xy(0);
		p.y = p_xy(1);
		p.z = 0;

		line_strip.points.push_back(p);
	}

	visualize_com_traj_pub_.publish(line_strip);
	visualize_com_traj_pub_.publish(traj_points);
	visualize_com_traj_pub_.publish(start_end_points);
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
		for (double t = leg_trajectories_[leg_i].start_time;
				t <= leg_trajectories_[leg_i].end_time; t += dt_)
		{
			Eigen::VectorXd pos_at_t = EvalLegPosAtT(t, leg_i);

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
	SetupComCmdTopics();
	SetupLegCmdTopics();
}

void MotionPlanner::SetupComCmdTopics()
{
  com_pos_traj_pub_ =
		ros_node_.advertise<std_msgs::Float64MultiArray>
		("com_pos_cmd", 10);

  com_vel_traj_pub_ =
		ros_node_.advertise<std_msgs::Float64MultiArray>
		("com_vel_cmd", 10);

  com_acc_traj_pub_ =
		ros_node_.advertise<std_msgs::Float64MultiArray>
		("com_acc_cmd", 10);
}

void MotionPlanner::SetupVisualizationTopics()
{
  visualize_com_traj_pub_ =
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

// ******************* //
// GAIT AND TRAJECTORY //
// ******************* //

void MotionPlanner::InitGaitSequence()
{
	// TODO: This should come from a ROS topic
	n_gait_steps_ = 20;
	t_per_gait_sequence_ = 10;
	t_per_step_ = t_per_gait_sequence_ / (double) n_gait_steps_;

	gait_sequence_.resize(n_legs_, n_gait_steps_);
	gait_sequence_ << 
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1,
		1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1,
		1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
}

int MotionPlanner::GetGaitStepFromTime(double t)
{
	double t_rel = std::fmod(t, t_per_gait_sequence_);
		
	int gait_step_i = t_rel / t_per_step_;
	return gait_step_i;
}

void MotionPlanner::GenerateSupportPolygons()
{
	support_polygons_.clear();

	std::vector<Eigen::Vector2d> new_polygon;
	for (int gait_step_i = 0; gait_step_i < n_gait_steps_; ++gait_step_i)
	{
		new_polygon.clear();
		auto curr_stance = stance_sequence_[gait_step_i];

		std::vector<int> leg_visualization_order = {0,2,3,1}; 
		for (int leg_i : leg_visualization_order)	
		{
			if (gait_sequence_(leg_i, gait_step_i) == 1)	
			{
				new_polygon.push_back(curr_stance.col(leg_i));
			}
		}
		support_polygons_.push_back(new_polygon);
	}
}

std::vector<Eigen::MatrixXd> MotionPlanner::GenerateStanceSequence(
		const Eigen::VectorXd &vel_cmd,
		const Eigen::MatrixXd &current_stance
		)
{
	std::vector<Eigen::MatrixXd> stance_sequence;
	stance_sequence.push_back(current_stance);

	for (int gait_step_i = 1; gait_step_i < n_gait_steps_; ++gait_step_i)
	{
		Eigen::MatrixXd next_stance =
			GenerateStanceForNextTimestep(
					vel_cmd, gait_step_i, stance_sequence[gait_step_i - 1]
					);
		stance_sequence.push_back(next_stance);
	}
	return stance_sequence;
}

Eigen::MatrixXd MotionPlanner::GenerateStanceForNextTimestep(
		const Eigen::VectorXd &vel_cmd,
		const int gait_step_i,
		const Eigen::MatrixXd &prev_stance
		)
{
	Eigen::MatrixXd next_stance(k2D, kNumLegs);
	next_stance.setZero();

	for (int foot_i = 0; foot_i < kNumLegs; ++foot_i)
	{
		Eigen::MatrixXd prev_foot_pos = prev_stance.col(foot_i);

		Eigen::MatrixXd next_foot_pos(k2D,1);
		if (gait_sequence_(foot_i, gait_step_i) == 1)
		{
			next_foot_pos = prev_foot_pos;
		}
		else
		{
			next_foot_pos = prev_foot_pos
				+ vel_cmd * t_per_step_;
		}
		
		next_stance.col(foot_i) = next_foot_pos;
	}

	return next_stance;
}


// ******************** //
// OPTIMIZATION PROBLEM //
// ******************** //

void MotionPlanner::SetupOptimizationProgram()
{
	InitDecisionVariables();
	InitMonomials();
	AddAccelerationCost();
	AddContinuityConstraints();
	AddInitialAndFinalConstraint();
	// Enforce zmp constraint
	// TODO: Implement

}

void MotionPlanner::InitMonomials()
{
	// Build monomial basis
	m_.resize(degree_ + 1);
	for (int d = 0; d < degree_ + 1; ++d)
	{
		m_(d) = drake::symbolic::Monomial(t_, d).ToExpression();
	}
	m_dot_ = drake::symbolic::Jacobian(m_, {t_});
	m_ddot_ = drake::symbolic::Jacobian(m_dot_, {t_});
}

void MotionPlanner::InitDecisionVariables()
{
	// traj is defined as p = p(t_)
	t_ = prog_.NewIndeterminates(1, 1, "t")(0,0);

	// Construct coefficients
	for (int k = 0; k < n_traj_segments_; ++k)
	{
		auto coeffs_at_k = prog_.NewContinuousVariables(
				traj_dimension_, degree_ + 1, "C"
				);

		coeffs_.push_back(coeffs_at_k);
	}
}

void MotionPlanner::AddAccelerationCost()
{
	double dt = 0.1;
	for (int k = 0; k < n_traj_segments_; ++k)
	{
		// Implement acceleration integral as sum
		for (double t = 0; t < 1; t += dt)
		{
			// TODO: Use the GetAccAtT function for this?
			symbolic_matrix_t T_ddot =
				GetTransformationMatrixAtT(t, m_ddot_)
				.cast <drake::symbolic::Expression>();
			symbolic_matrix_t Q = T_ddot.transpose() * T_ddot;

			auto coeffs_test = coeffs_[k];
			Eigen::Map<symbolic_vector_t> alpha(
					coeffs_test.data(), coeffs_test.size()
					);

			drake::symbolic::Expression cost_at_t =
				dt * alpha.transpose() * Q * alpha;

			prog_.AddQuadraticCost(cost_at_t);
		}
	}
}

void MotionPlanner::AddContinuityConstraints()
{
	for (int k = 0; k < n_traj_segments_ - 1; ++k)
	{
		prog_.AddLinearConstraint(
				GetPosExpressionAtT(1, k).array()
				== GetPosExpressionAtT(0, k + 1).array()
				);
		prog_.AddLinearConstraint(
				GetVelExpressionAtT(1, k).array()
				== GetVelExpressionAtT(0, k + 1).array()
				);
	}
}

void MotionPlanner::AddInitialAndFinalConstraint()
{
	pos_initial_ = GetPolygonCentroid(support_polygons_[0]);
	pos_final_ = GetPolygonCentroid(support_polygons_.back());

	prog_.AddLinearConstraint(
			GetPosExpressionAtT(0, 0).array()
			== pos_initial_.array()
			);
	prog_.AddLinearConstraint(
			GetPosExpressionAtT(1, n_traj_segments_ - 1).array()
			== pos_final_.array()
			);
}

void MotionPlanner::GeneratePolynomialsFromSolution()
{
	const std::vector<symbolic_matrix_t> coeff_values = GetCoeffValues();

	polynomials_pos_ = GeneratePolynomials(m_, coeff_values);
	polynomials_vel_ = GeneratePolynomials(m_dot_, coeff_values);
	polynomials_acc_ = GeneratePolynomials(m_ddot_, coeff_values);
}

std::vector<symbolic_matrix_t> MotionPlanner::GetCoeffValues()
{
	std::vector<symbolic_matrix_t> coeff_values;
	for (int k = 0; k < n_traj_segments_; ++k)
	{
		symbolic_matrix_t coeff_values_for_curr_segment =
			result_.GetSolution(coeffs_[k]);
		coeff_values.push_back(coeff_values_for_curr_segment);
	}
	return coeff_values;
}

polynomial_matrix_t MotionPlanner::GeneratePolynomials(
		const symbolic_vector_t &monomial_basis,
		const std::vector<symbolic_matrix_t> &coeff_values 
		)
{
	polynomial_matrix_t polynomials;
	polynomials.resize(traj_dimension_, n_traj_segments_);

	for (int k = 0; k < n_traj_segments_; ++k)
	{
		symbolic_vector_t polynomial = coeff_values[k] * monomial_basis;

		for (int dim = 0; dim < traj_dimension_; ++dim)
			polynomials(dim,k) =
				drake::symbolic::Polynomial(polynomial[dim]);
	}

	return polynomials;
}


// **************** //
// HELPER FUNCTIONS //
// **************** //

Eigen::MatrixXd MotionPlanner::GetTransformationMatrixAtT(
		double t, symbolic_vector_t v
		)
{
	// Evaluate monomial at t
	drake::symbolic::Environment t_curr {{t_, t}};
	Eigen::VectorXd v_at_t(v.rows());
	for (int i = 0; i < v.rows(); ++i)
		v_at_t(i) = v(i).Evaluate(t_curr);

	// Construct transformation at t
	Eigen::MatrixXd T_at_t(traj_dimension_, v.rows() * 2);
	T_at_t.setZero();
	for (int dim = 0; dim < traj_dimension_; ++dim)
		T_at_t.block(dim, dim * v.rows(), 1, v.rows()) = v_at_t.transpose();

	return T_at_t;
}

symbolic_vector_t MotionPlanner::GetPosExpressionAtT(double t)
{
	int traj_segment_index = 0;
	while ((double) traj_segment_index + 1 < t) ++traj_segment_index;
	double t_in_segment = t - (double) traj_segment_index;
	auto pos = GetTrajExpressionAtT(t_in_segment, m_, traj_segment_index);
	return pos;
}

symbolic_vector_t MotionPlanner::GetPosExpressionAtT(
		double t_in_segment, int segment_j
		)
{
	auto pos = GetTrajExpressionAtT(t_in_segment, m_, segment_j);
	return pos;
}

symbolic_vector_t MotionPlanner::GetVelExpressionAtT(
		double t_in_segment, int segment_j
		)
{
	auto vel = GetTrajExpressionAtT(t_in_segment, m_dot_, segment_j);
	return vel;
}

symbolic_vector_t MotionPlanner::GetAccExpressionAtT(
		double t_in_segment, int segment_j
		)
{
	auto acc = GetTrajExpressionAtT(t_in_segment, m_ddot_, segment_j);
	return acc;
}

symbolic_vector_t MotionPlanner::GetTrajExpressionAtT(
		double t, symbolic_vector_t v, int segment_j
		)
{
	symbolic_matrix_t T =
		GetTransformationMatrixAtT(t, v)
		.cast <drake::symbolic::Expression>();

	Eigen::Map<symbolic_vector_t> alpha(
			coeffs_[segment_j].data(), coeffs_[segment_j].size()
			);

	symbolic_vector_t traj_at_t = T * alpha;
	return traj_at_t;
}

// TODO: For now, this only calculates the center of mass which may not coincide with the actual centroid
Eigen::Vector2d MotionPlanner::GetPolygonCentroid(
		std::vector<Eigen::Vector2d> polygon
		)
{
	Eigen::Vector2d centroid(0,0);
	for (int point_i = 0; point_i < polygon.size(); ++point_i)
	{
		centroid += polygon[point_i];
	}
	centroid /= polygon.size();

	return centroid;
}

int main( int argc, char** argv )
{

	int traj_degree = 5;
	int n_traj_segments = 10;

  ros::init(argc, argv, "motion_planner");
	MotionPlanner planner(traj_degree, n_traj_segments);
	planner.GenerateTrajectory();

  ros::Rate r(30);
  while (ros::ok())
  {
		const double time = ros::Time::now().toSec();
		planner.PublishLegTrajectories(time);
		planner.PublishComTrajectories(time);
		planner.PublishLegTrajectoriesVisualization();
		planner.PublishComTrajVisualization();
		planner.PublishPolygonVisualizationAtTime(time);
    r.sleep();
  }
}
