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
	gait_sequence_ = CreateCrawlSequence();

	// TODO: Remember to update robot dynamcis with gen coords
	robot_dynamics_.SetStateDefault();
	current_stance_ = robot_dynamics_.GetStacked2DFootPosInW();
	// TODO: For now, this assumes that we always start at the first gait step

	leg_planner_.SetGaitSequence(gait_sequence_);
	leg_planner_.PlanLegsMotion(vel_cmd_, current_stance_);

	// COM optimization
	SetupOptimizationProgram(); // TODO: move this to the right spot
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
			EvalComPosAtT(time), com_pos_cmd
			);
	com_pos_traj_pub_.publish(com_pos_cmd);
}

void MotionPlanner::PublishComVelCmd(const double time)
{
	std_msgs::Float64MultiArray com_vel_cmd;
	tf::matrixEigenToMsg(
			EvalComVelAtT(time), com_vel_cmd
			);
	com_vel_traj_pub_.publish(com_vel_cmd);
}

void MotionPlanner::PublishComAccCmd(const double time)
{
	std_msgs::Float64MultiArray com_acc_cmd;
	tf::matrixEigenToMsg(
			EvalComAccAtT(time), com_acc_cmd
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
	double t_rel = std::fmod(t, gait_sequence_.duration); // TODO: these should be replaced with a common time

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
		for (double t = leg_planner_.GetLegTrajStartTime(leg_i);
				t <= leg_planner_.GetLegTrajEndTime(leg_i); t += dt_)
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
	pos_initial_ = leg_planner_.GetFirstPolygonCentroid();
	pos_final_ = leg_planner_.GetLastPolygonCentroid();

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
