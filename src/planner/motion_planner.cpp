#include "planner/motion_planner.hpp"

// **************** // 
// PUBLIC FUNCTIONS //
// **************** // 

MotionPlanner::MotionPlanner(
		int degree,
		int n_traj_segments
		)
	:
		n_traj_segments_(n_traj_segments),
		degree_(degree)
{
	InitRos();
	SetupOptimizationProgram();
	AddTestPolygons();

}

Eigen::VectorXd MotionPlanner::EvalTrajAtT(double t)
{
	int traj_segment_index = 0;
	while ((double) traj_segment_index + 1 < t) ++traj_segment_index;

	double t_in_segment = t - (double) traj_segment_index;

	Eigen::VectorXd traj_value(traj_dimension_);
	drake::symbolic::Environment t_at_t {{t_, t_in_segment}};
	for (int dim = 0; dim < traj_dimension_; ++dim)
	{
		traj_value(dim) = 
			polynomials_(dim, traj_segment_index).Evaluate(t_at_t);
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
	GeneratePolynomials();
}

void MotionPlanner::PublishPolygons()
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

		polygons_pub_.publish(polygon_msg);
	}
}

void MotionPlanner::PublishTrajectory()
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
	start_end_points.scale.x = 0.1;
  start_end_points.scale.y = 0.1;
	start_end_points.color.r = 1.0;
	start_end_points.color.a = 1.0;

  traj_points.scale.x = 0.05;
  traj_points.scale.y = 0.05;
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
		Eigen::VectorXd p_xy = EvalTrajAtT(k);
		p.x = p_xy(0);
		p.y = p_xy(1);
		p.z = 0;

		traj_points.points.push_back(p);
	}

	for (double t = 0; t < n_traj_segments_ - dt_; t += dt_)
	{
		Eigen::VectorXd p_xy = EvalTrajAtT(t);

		p.x = p_xy(0);
		p.y = p_xy(1);
		p.z = 0;

		line_strip.points.push_back(p);
	}

	traj_pub_.publish(line_strip);
	traj_pub_.publish(traj_points);
	traj_pub_.publish(start_end_points);
}

// ***************** // 
// PRIVATE FUNCTIONS //
// ***************** // 

void MotionPlanner::GeneratePolynomials()
{
	polynomials_.resize(traj_dimension_, n_traj_segments_);
	for (int k = 0; k < n_traj_segments_; ++k)
	{
		symbolic_vector_t polynomial = result_
			.GetSolution(coeffs_[k]) * m_;

		for (int dim = 0; dim < traj_dimension_; ++dim)
			polynomials_(dim,k) = drake::symbolic::Polynomial(polynomial[dim]);
	}
}


// Constructs the transformation matrix
// T = [ m 0 0 0 ...
//		 [ 0 m 0 0 ...
//		 [ 0 0 m 0 ...
//		 [ 0 0 0 m ...
void MotionPlanner::construct_transform_matrix(
		symbolic_matrix_t *T, symbolic_vector_t *v
		)
{
	T->resize(traj_dimension_, v->rows() * 2);
	T->setZero();

	for (int dim = 0; dim < traj_dimension_; ++dim)
	{
		T->block(dim, dim * v->rows(), 1, v->rows()) = v->transpose();
	}
}

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

symbolic_vector_t MotionPlanner::GetPosAtT(double t)
{
	int traj_segment_index = 0;
	while ((double) traj_segment_index + 1 < t) ++traj_segment_index;
	double t_in_segment = t - (double) traj_segment_index;
	auto pos = GetTrajExpressionAtT(t_in_segment, m_, traj_segment_index);
	return pos;
}

symbolic_vector_t MotionPlanner::GetPosAtT(double t_in_segment, int segment_j)
{
	auto pos = GetTrajExpressionAtT(t_in_segment, m_, segment_j);
	return pos;
}

symbolic_vector_t MotionPlanner::GetVelAtT(double t_in_segment, int segment_j)
{
	auto vel = GetTrajExpressionAtT(t_in_segment, m_dot_, segment_j);
	return vel;
}

symbolic_vector_t MotionPlanner::GetAccAtT(double t_in_segment, int segment_j)
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
//	std::cout << "T: " << std::endl
//		<< T << std::endl << std::endl;

	Eigen::Map<symbolic_vector_t> alpha(
			coeffs_[segment_j].data(), coeffs_[segment_j].size()
			);

//	std::cout << "coeffs_[j]: " << std::endl
//		<< coeffs_[segment_j] << std::endl << std::endl;
//
//	std::cout << "alpha: " << std::endl
//		<< alpha << std::endl << std::endl;

	symbolic_vector_t traj_at_t = T * alpha;
	return traj_at_t;
}

void MotionPlanner::SetupOptimizationProgram()
{
	InitDecisionVariables();
	AddAccelerationCost();
	AddContinuityConstraints();
	AddTestPolygons();
	AddInitialAndFinalConstraint();

	// Enforce zmp constraint
	// TODO: Implement

}

void MotionPlanner::InitDecisionVariables()
{
	// traj is defined as p = p(t_)
	t_ = prog_.NewIndeterminates(1, 1, "t")(0,0);

	// Build monomial basis
	m_.resize(degree_ + 1);
	for (int d = 0; d < degree_ + 1; ++d)
	{
		m_(d) = drake::symbolic::Monomial(t_, d).ToExpression();
	}
	m_dot_ = drake::symbolic::Jacobian(m_, {t_});
	m_ddot_ = drake::symbolic::Jacobian(m_dot_, {t_});

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
				GetPosAtT(1, k).array() == GetPosAtT(0, k + 1).array()
				);
		prog_.AddLinearConstraint(
				GetVelAtT(1, k).array() == GetVelAtT(0, k + 1).array()
				);
	}
}

void MotionPlanner::AddInitialAndFinalConstraint()
{
	// TODO: Factor this into separate function
	pos_initial_ = Eigen::Vector2d(0,0);
	for (int point_i = 0; point_i < support_polygons_[0].size(); ++point_i)
	{
		pos_initial_ += support_polygons_[0][point_i];
	}
	pos_initial_ /= support_polygons_[0].size();

	pos_final_ = Eigen::Vector2d(0,0);
	for (int point_i = 0; point_i < support_polygons_.back().size(); ++point_i)
	{
		pos_final_ += support_polygons_.back()[point_i];
	}
	pos_final_ /= support_polygons_.back().size();

	std::cout << "initial: " << std::endl << pos_initial_ << std::endl << std::endl;
	std::cout << "final: " << std::endl << pos_final_ << std::endl << std::endl;

//	prog_.AddLinearConstraint(
//			GetPosAtT(4.0).array() == Eigen::Vector2d(4,2).array()
//			);
	prog_.AddLinearConstraint(
			GetPosAtT(0, 0).array() == pos_initial_.array()
			);
	prog_.AddLinearConstraint(
			GetPosAtT(1, n_traj_segments_ - 1).array() == pos_final_.array()
			);
}

void MotionPlanner::AddTestPolygons()
{
	// TODO: Only for testing
	std::vector<Eigen::Vector2d> polygon;
	polygon.push_back(Eigen::Vector2d(0,0));
	polygon.push_back(Eigen::Vector2d(0,3));
	polygon.push_back(Eigen::Vector2d(2,0));
	support_polygons_.push_back(polygon);

	polygon.clear();
	polygon.push_back(Eigen::Vector2d(0,1));
	polygon.push_back(Eigen::Vector2d(2.5,1.5));
	polygon.push_back(Eigen::Vector2d(2,3.5));
	support_polygons_.push_back(polygon);

	polygon.clear();
	polygon.push_back(Eigen::Vector2d(0,2));
	polygon.push_back(Eigen::Vector2d(2,2));
	polygon.push_back(Eigen::Vector2d(0,5));
	support_polygons_.push_back(polygon);

	polygon.clear();
	polygon.push_back(Eigen::Vector2d(0,2));
	polygon.push_back(Eigen::Vector2d(2.5,3.5));
	polygon.push_back(Eigen::Vector2d(2,4.5));
	support_polygons_.push_back(polygon);

	polygon.clear();
	polygon.push_back(Eigen::Vector2d(0,3));
	polygon.push_back(Eigen::Vector2d(2,3));
	polygon.push_back(Eigen::Vector2d(0,6));
	support_polygons_.push_back(polygon);
}

void MotionPlanner::InitRos()
{
  traj_pub_ =
		ros_node_.advertise<visualization_msgs::Marker>
		("visualization_trajectory", 10);

  polygons_pub_ =
		ros_node_.advertise<visualization_msgs::Marker>
		("visualization_polygons", 10);
}

