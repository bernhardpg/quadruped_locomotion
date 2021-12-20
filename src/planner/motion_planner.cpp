#include "planner/motion_planner.hpp"

MotionPlanner::MotionPlanner(
		int degree,
		int n_traj_segments
		)
	:
		n_traj_segments_(n_traj_segments),
		degree_(degree)
{
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

	//	TODO: I may not need these as symbolic after all?
//	construct_transform_matrix(&T_, &m_);
//	construct_transform_matrix(&T_dot_, &m_dot_);
//	construct_transform_matrix(&T_ddot_, &m_ddot_);

	// Add cost on acceleration
	double dt = 0.1;

	for (int k = 0; k < n_traj_segments; ++k)
	{
		// Implement integral over segment as sum
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

	// Enforce continuity for position and velocity
	for (int k = 0; k < n_traj_segments - 1; ++k)
	{
		prog_.AddLinearConstraint(
				GetPosAtT(1, k).array() == GetPosAtT(0, k + 1).array()
				);
		prog_.AddLinearConstraint(
				GetVelAtT(1, k).array() == GetVelAtT(0, k + 1).array()
				);
	}
	
	// Enforce zmp constraint

	// Initial and final conditions
	// TODO: These are just for testing purposes
	Eigen::Vector2d pos_initial(0,0);
	Eigen::Vector2d pos_final(5,5);

	prog_.AddLinearConstraint(GetPosAtT(0, 0) == pos_initial);
	prog_.AddLinearConstraint(
			GetPosAtT(1, n_traj_segments_ - 1) == pos_final
			);

	// Solve
	result_ = Solve(prog_);
	std::cout << "Solver id: " << result_.get_solver_id()
		<< "\nFound solution: " << result_.is_success()
		<< "\nSolution result: " << result_.get_solution_result()
		<< std::endl;
	assert(result_.is_success());

	GeneratePolynomials();
	std::cout << "Traj at t = 1.5\n"
		<< EvalTrajAtT(1.5) << std::endl;
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

symbolic_vector_t MotionPlanner::GetPosAtT(double t, int segment_j)
{
	auto pos = EvalTrajectoryAtT(t, m_, segment_j);
	return pos;
}

symbolic_vector_t MotionPlanner::GetVelAtT(double t, int segment_j)
{
	auto vel = EvalTrajectoryAtT(t, m_dot_, segment_j);
	return vel;
}

symbolic_vector_t MotionPlanner::GetAccAtT(double t, int segment_j)
{
	auto acc = EvalTrajectoryAtT(t, m_ddot_, segment_j);
	return acc;
}

symbolic_vector_t MotionPlanner::EvalTrajectoryAtT(
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
