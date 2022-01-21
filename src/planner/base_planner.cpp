#include "planner/base_planner.hpp"

// *************** //
// WALK TRAJECTORY //
// *************** //

BasePlanner::BasePlanner()
{
	const int kDefaultPolynomialDegree = 5;
	polynomial_degree_ = kDefaultPolynomialDegree;

	const int kDefaultNumTrajSegments = 10;
	n_traj_segments_ = kDefaultNumTrajSegments;
}

void BasePlanner::PlanBaseWalkMotion(const double walking_height)
{
	walking_height_ = walking_height;

	FormulateOptimizationProblem();
	GenerateWalkTrajectory();
}

void BasePlanner::SetGaitSequence(GaitSequence gait_sequence)
{
	gait_sequence_ = gait_sequence;

	seconds_per_segment_ =
		gait_sequence_.duration / (double) n_traj_segments_;
}

void BasePlanner::SetSupportPolygons(
		const std::vector<std::vector<Eigen::Vector2d>> &support_polygons
		)
{
	support_polygons_ = support_polygons;
}

void BasePlanner::SetCurrPos(const Eigen::Vector2d &curr_2d_pos)
{
	curr_2d_pos_ = curr_2d_pos;
}

Eigen::VectorXd BasePlanner::EvalWalkTrajPosAtT(const double t)
{
	return EvalWalkTrajAtT(t, 0);
}

Eigen::VectorXd BasePlanner::EvalWalkTrajVelAtT(const double t)
{
	return EvalWalkTrajAtT(t, 1);
}

Eigen::VectorXd BasePlanner::EvalWalkTrajAccAtT(const double t)
{
	return EvalWalkTrajAtT(t, 2);
}

const int BasePlanner::GetNumTrajSegments()
{
	return n_traj_segments_;
}

const double BasePlanner::GetSecondsPerSegment()
{
	return seconds_per_segment_;
}

void BasePlanner::GeneratePolynomialsFromSolution()
{
	const std::vector<symbolic_matrix_t> coeff_values = GetCoeffValues();

	polynomials_pos_ = GeneratePolynomials(m_, coeff_values);
	polynomials_vel_ = GeneratePolynomials(m_dot_, coeff_values);
	polynomials_acc_ = GeneratePolynomials(m_ddot_, coeff_values);
}

std::vector<symbolic_matrix_t> BasePlanner::GetCoeffValues()
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

polynomial_matrix_t BasePlanner::GeneratePolynomials(
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

Eigen::VectorXd BasePlanner::EvalWalkTrajAtT(
		const double t_abs, const int derivative
		)
{
	int traj_segment_index = GetSegmentIndexForT(t_abs);
	double t_normalized =
		GetTimeInSegmentNormalized(t_abs, traj_segment_index);

	Eigen::VectorXd traj_value_2d(traj_dimension_);
	drake::symbolic::Environment t_at_t {{t_, t_normalized}};
	double z_coord;
	for (int dim = 0; dim < traj_dimension_; ++dim)
	{
		switch(derivative)
		{
			case 0:
				traj_value_2d(dim) = 
					polynomials_pos_(dim, traj_segment_index).Evaluate(t_at_t);
				z_coord = walking_height_;
				break;
			case 1:
				traj_value_2d(dim) = 
					polynomials_vel_(dim, traj_segment_index).Evaluate(t_at_t);
				z_coord = 0;
				break;
			case 2:
				traj_value_2d(dim) = 
					polynomials_acc_(dim, traj_segment_index).Evaluate(t_at_t);
				z_coord = 0;
				break;
		}
	}

	Eigen::VectorXd traj_value(k3D);
	traj_value <<
		traj_value_2d(0),
		traj_value_2d(1),
		z_coord;

	return traj_value;
}


// ******************** //
// OPTIMIZATION PROBLEM //
// ******************** //

void BasePlanner::GenerateWalkTrajectory()
{
	SolveOptimization();
	GeneratePolynomialsFromSolution();
}

void BasePlanner::SolveOptimization()
{
	//std::cout << prog_->to_string();
	result_ = drake::solvers::Solve(*prog_);

	std::cout << "Solver id: " << result_.get_solver_id()
		<< "\nFound solution: " << result_.is_success()
		<< "\nSolution result: " << result_.get_solution_result()
		<< std::endl;
	assert(result_.is_success());
}

void BasePlanner::FormulateOptimizationProblem()
{
	CreateNewMathProg();
	InitDecisionVariables();
	InitMonomials();
	AddAccelerationCost();
	AddContinuityConstraints();
	AddInitialPosConstraint();
	//AddFinalPosConstraint();
	AddZmpConstraints();
}

void BasePlanner::AddZmpConstraints()
{
	CreatePolygonIneqForm();

	// TODO: move this?
	const double dt = 0.1;

	const double end_time = gait_sequence_.duration;

	for (double t = 0; t < end_time; t += dt)
	{
		auto zmp_at_t = GetZmpExpressionAtT(t);

		const int polygon_i = GetGaitStepFromTime(t);
		const Eigen::MatrixXd &A = polygons_A_matrices_[polygon_i];
		const Eigen::VectorXd &b = polygons_b_vectors_[polygon_i];
		const Eigen::VectorXd zero = Eigen::VectorXd::Zero(b.rows());

		prog_->AddLinearConstraint(
				(A * zmp_at_t + b).array() >= zero.array()
				);
	}
}

symbolic_vector_t BasePlanner::GetZmpExpressionAtT(const double t_abs)
{
	symbolic_vector_t pos_at_t = GetPosExpressionAtT(t_abs);
	symbolic_vector_t acc_at_t = GetAccExpressionAtT(t_abs);
	const double g = 9.81;

	// Assumes z_ddot = 0;
	symbolic_vector_t zmp_at_t = pos_at_t - walking_height_ * acc_at_t / g;
	return zmp_at_t;
}

void BasePlanner::CreatePolygonIneqForm()
{
	polygons_A_matrices_.clear();
	polygons_b_vectors_.clear();

	for (int polygon_i = 0;
			polygon_i < support_polygons_.size();
			++polygon_i)
	{
		const std::vector<Eigen::Vector2d> &support_polygon =
			support_polygons_[polygon_i];
		Eigen::MatrixXd A = ConstructAMatrixFromPolygon(support_polygon);
		polygons_A_matrices_.push_back(A);

		Eigen::VectorXd b = ConstructBVectorFromPolygon(support_polygon, A);
		polygons_b_vectors_.push_back(b);
	}
}

Eigen::MatrixXd BasePlanner::ConstructAMatrixFromPolygon(
		const std::vector<Eigen::Vector2d> &points
		)
{
	const int num_edges = points.size() - 1;

	Eigen::MatrixXd A(num_edges, k2D);
	A.setZero();

	for (int i = 0; i < num_edges; ++i)
	{
		Eigen::Vector2d p1 = points[i];
		Eigen::Vector2d p2 = points[i + 1];
		Eigen::Vector2d n = GetNormalPointingInwards(p1, p2);
		A.row(i) = n;
	}

	return A;
}

Eigen::VectorXd BasePlanner::ConstructBVectorFromPolygon(
		const std::vector<Eigen::Vector2d> &points,
		const Eigen::MatrixXd &A
		)
{
	const int num_edges = points.size() - 1;
	const double kSafetyMargin = 0.0;

	Eigen::VectorXd b(num_edges);
	b.setZero();

	for (int i = 0; i < num_edges; ++i)
	{
		const Eigen::Vector2d n = A.row(i);
		const Eigen::Vector2d p = points[i];
		b(i) = - n.transpose() * p;
	}

	b *= 0.5;

	return b;
}

Eigen::Vector2d BasePlanner::GetNormalPointingInwards(
		const Eigen::Vector2d &p1,
		const Eigen::Vector2d &p2
		)
{
	const Eigen::Vector2d r = p2 - p1;
	Eigen::Vector2d n(-r(1), r(0));
	n.normalize();

	return n;
}

void BasePlanner::CreateNewMathProg()
{
	prog_ = std::unique_ptr<drake::solvers::MathematicalProgram>(
			new drake::solvers::MathematicalProgram
			);
}

void BasePlanner::InitDecisionVariables()
{
	// traj is defined as p = p(t_)
	t_ = prog_->NewIndeterminates(1, 1, "t")(0,0);

	// Construct coefficients
	coeffs_.clear();
	for (int k = 0; k < n_traj_segments_; ++k)
	{
		auto coeffs_at_k = prog_->NewContinuousVariables(
				traj_dimension_, polynomial_degree_ + 1, "C"
				);

		coeffs_.push_back(coeffs_at_k);
	}
}

void BasePlanner::InitMonomials()
{
	// Build monomial basis
	m_.resize(polynomial_degree_ + 1);
	for (int d = 0; d < polynomial_degree_ + 1; ++d)
	{
		m_(d) = drake::symbolic::Monomial(t_, d).ToExpression();
	}
	m_dot_ = drake::symbolic::Jacobian(m_, {t_});
	m_ddot_ = drake::symbolic::Jacobian(m_dot_, {t_});
}

void BasePlanner::AddAccelerationCost()
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

			Eigen::Map<symbolic_vector_t> alpha(
					coeffs_[k].data(), coeffs_[k].size()
					);

			drake::symbolic::Expression cost_at_t =
				dt * alpha.transpose() * Q * alpha;

			const bool is_convex = true;
			prog_->AddQuadraticCost(cost_at_t, is_convex);
		}
	}
}

void BasePlanner::AddContinuityConstraints()
{
	const double t_segment_start = 0;
	const double t_segment_end = 1;

	for (int k = 0; k < n_traj_segments_ - 1; ++k)
	{
		prog_->AddLinearConstraint(
				GetPosExpressionAtT(t_segment_end, k).array()
				== GetPosExpressionAtT(t_segment_start, k + 1).array()
				);
		prog_->AddLinearConstraint(
				GetVelExpressionAtT(t_segment_end, k).array()
				== GetVelExpressionAtT(t_segment_start, k + 1).array()
				);
	}
}

void BasePlanner::AddInitialPosConstraint()
{
	symbolic_vector_t pos_initial_expr = GetPosExpressionAtT(0);
	prog_->AddLinearConstraint(
			pos_initial_expr.array() == curr_2d_pos_.array()
			);
}

void BasePlanner::AddFinalPosConstraint()
{
	Eigen::Vector2d pos_final_desired = GetPolygonCentroid(
			support_polygons_.back()
			);
	const double end_time = gait_sequence_.duration;
	symbolic_vector_t pos_final_expr = GetPosExpressionAtT(end_time);
	prog_->AddLinearConstraint(
			pos_final_expr.array() == pos_final_desired.array()
			);
}

// ******* //
// STANDUP //
// ******* //

void BasePlanner::PlanBaseStandupMotion(
		const double seconds_to_standup_config,
		const double target_height,
		const Eigen::MatrixXd &curr_pose
		)
{
	seconds_to_standup_ = seconds_to_standup_config;

	const std::vector<double> breaks =
	{ 0.0, seconds_to_standup_config };

	Eigen::MatrixXd target_pose(k3D,1);
	target_pose << curr_pose(0),
								 curr_pose(1),
								 target_height;

	const std::vector<Eigen::MatrixXd> samples =
	{ curr_pose, target_pose};

	standup_traj_pos_ =
		drake::trajectories::PiecewisePolynomial<double>
					::FirstOrderHold(breaks, samples);
	standup_traj_vel_	= standup_traj_pos_.derivative(1);
	standup_traj_acc_ = standup_traj_vel_.derivative(1);
}

Eigen::VectorXd BasePlanner::EvalStandupPosTrajAtT(const double time)
{
	if (time > seconds_to_standup_)
		return standup_traj_pos_.value(seconds_to_standup_);
	return standup_traj_pos_.value(time);
}

Eigen::VectorXd BasePlanner::EvalStandupVelTrajAtT(const double time)
{
	if (time > seconds_to_standup_)
		return Eigen::VectorXd::Zero(k3D);
	return standup_traj_vel_.value(time);
}

Eigen::VectorXd BasePlanner::EvalStandupAccTrajAtT(const double time)
{
	if (time > seconds_to_standup_)
		return Eigen::VectorXd::Zero(k3D);
	return standup_traj_acc_.value(time);
}

// **************** //
// HELPER FUNCTIONS //
// **************** //

double BasePlanner::GetTimeInSegmentNormalized(
		const double t_abs, const int traj_segment_index
		)
{
	double t_in_segment =
		t_abs - (double) traj_segment_index * seconds_per_segment_;
	double t_normalized = t_in_segment / seconds_per_segment_;
	return t_normalized;
}

int BasePlanner::GetSegmentIndexForT(double t)
{
	int traj_segment_index = 0;
	while (
			(double) (traj_segment_index + 1) * seconds_per_segment_ < t
			)
		++traj_segment_index;
	return traj_segment_index;
}

Eigen::MatrixXd BasePlanner::GetTransformationMatrixAtT(
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

symbolic_vector_t BasePlanner::GetPosExpressionAtT(const double t_abs)
{
	int traj_segment_index = GetSegmentIndexForT(t_abs);
	double t_normalized =
		GetTimeInSegmentNormalized(t_abs, traj_segment_index);

	auto pos = GetTrajExpressionAtT(t_normalized, m_, traj_segment_index);
	return pos;
}

symbolic_vector_t BasePlanner::GetAccExpressionAtT(const double t_abs)
{
	int traj_segment_index = GetSegmentIndexForT(t_abs);
	double t_normalized =
		GetTimeInSegmentNormalized(t_abs, traj_segment_index);

	auto acc =
		GetTrajExpressionAtT(t_normalized, m_ddot_, traj_segment_index);
	return acc;
}

symbolic_vector_t BasePlanner::GetPosExpressionAtT(
		double t_in_segment_normalized, int segment_j
		)
{
	auto pos =
		GetTrajExpressionAtT(t_in_segment_normalized, m_, segment_j);
	return pos;
}

symbolic_vector_t BasePlanner::GetVelExpressionAtT(
		double t_in_segment_normalized, int segment_j
		)
{
	auto vel =
		GetTrajExpressionAtT(t_in_segment_normalized, m_dot_, segment_j);
	return vel;
}

symbolic_vector_t BasePlanner::GetAccExpressionAtT(
		double t_in_segment_normalized, int segment_j
		)
{
	auto acc =
		GetTrajExpressionAtT(t_in_segment_normalized, m_ddot_, segment_j);
	return acc;
}

symbolic_vector_t BasePlanner::GetTrajExpressionAtT(
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

// TODO: This only calculates center of mass and not actually centroid
Eigen::Vector2d BasePlanner::GetPolygonCentroid(
		const std::vector<Eigen::Vector2d> &polygon
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

// TODO: currently this is duplicated both here and in LegPlanner
int BasePlanner::GetGaitStepFromTime(const double t_abs)
{
	double t_rel = std::fmod(t_abs, gait_sequence_.duration);
		
	int gait_step_i = t_rel / gait_sequence_.step_time;
	return gait_step_i;
}

