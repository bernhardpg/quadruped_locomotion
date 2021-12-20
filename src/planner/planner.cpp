#include "planner/planner.hpp"

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

	construct_transform_matrix(&T_, &m_);
	construct_transform_matrix(&T_dot_, &m_dot_);
	construct_transform_matrix(&T_ddot_, &m_ddot_);

	// Add cost on acceleration

	// Enforce continuity
	
	// Enforce zmp constraint

	// Solve
	//Solve(prog_);
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
