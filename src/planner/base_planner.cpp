#include "planner/base_planner.hpp"

void BasePlanner::PlanBaseMotion(
		const int polynomial_degree, const int n_traj_segments
		)
{
	n_traj_segments_ = n_traj_segments;
	polynomial_degree_ = polynomial_degree;

	FormulateOptimizationProblem();
	GenerateTrajectory();
}

void BasePlanner::SetSupportPolygons(
		const std::vector<std::vector<Eigen::Vector2d>> &support_polygons
		)
{
	support_polygons_ = support_polygons;
}

Eigen::VectorXd BasePlanner::EvalBasePosAtT(const double t)
{
	return EvalBaseTrajAtT(t, 0);
}

Eigen::VectorXd BasePlanner::EvalBaseVelAtT(const double t)
{
	return EvalBaseTrajAtT(t, 1);
}

Eigen::VectorXd BasePlanner::EvalBaseAccAtT(const double t)
{
	return EvalBaseTrajAtT(t, 2);
}

const int BasePlanner::GetNumTrajSegments()
{
	return n_traj_segments_;
}

// ********** //
// TRAJECTORY //
// ********** //

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

Eigen::VectorXd BasePlanner::EvalBaseTrajAtT(
		const double t, const int derivative
		)
{
	// TODO
	double t_rel = t; //std::fmod(t, gait_sequence_.duration); // TODO: these should be replaced with a basemon time

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


// ******************** //
// OPTIMIZATION PROBLEM //
// ******************** //

void BasePlanner::FormulateOptimizationProblem()
{
	InitDecisionVariables();
	InitMonomials();
	AddAccelerationCost();
	AddContinuityConstraints();
	AddInitialAndFinalConstraint();
	// Enforce zmp constraint
	// TODO: Implement
}

void BasePlanner::GenerateTrajectory()
{
	SolveOptimization();
	GeneratePolynomialsFromSolution();
}

void BasePlanner::SolveOptimization()
{
	result_ = drake::solvers::Solve(prog_);
//	ROS_INFO_STREAM("Solver id: " << result_.get_solver_id()
//		<< "\nFound solution: " << result_.is_success()
//		<< "\nSolution result: " << result_.get_solution_result()
//		<< std::endl);
	assert(result_.is_success());
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

void BasePlanner::InitDecisionVariables()
{
	// traj is defined as p = p(t_)
	t_ = prog_.NewIndeterminates(1, 1, "t")(0,0);

	// Construct coefficients
	for (int k = 0; k < n_traj_segments_; ++k)
	{
		auto coeffs_at_k = prog_.NewContinuousVariables(
				traj_dimension_, polynomial_degree_ + 1, "C"
				);

		coeffs_.push_back(coeffs_at_k);
	}
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

void BasePlanner::AddContinuityConstraints()
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

void BasePlanner::AddInitialAndFinalConstraint()
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

// **************** //
// HELPER FUNCTIONS //
// **************** //

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

symbolic_vector_t BasePlanner::GetPosExpressionAtT(double t)
{
	int traj_segment_index = 0;
	while ((double) traj_segment_index + 1 < t) ++traj_segment_index;
	double t_in_segment = t - (double) traj_segment_index;
	auto pos = GetTrajExpressionAtT(t_in_segment, m_, traj_segment_index);
	return pos;
}

symbolic_vector_t BasePlanner::GetPosExpressionAtT(
		double t_in_segment, int segment_j
		)
{
	auto pos = GetTrajExpressionAtT(t_in_segment, m_, segment_j);
	return pos;
}

symbolic_vector_t BasePlanner::GetVelExpressionAtT(
		double t_in_segment, int segment_j
		)
{
	auto vel = GetTrajExpressionAtT(t_in_segment, m_dot_, segment_j);
	return vel;
}

symbolic_vector_t BasePlanner::GetAccExpressionAtT(
		double t_in_segment, int segment_j
		)
{
	auto acc = GetTrajExpressionAtT(t_in_segment, m_ddot_, segment_j);
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

