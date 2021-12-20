#pragma once

#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>
#include <Eigen/Core>

typedef Eigen::Matrix<drake::symbolic::Expression, Eigen::Dynamic, Eigen::Dynamic> symbolic_matrix_t;

typedef Eigen::Matrix<drake::symbolic::Expression, Eigen::Dynamic, 1> symbolic_vector_t;

class MotionPlanner
{
	public:
		MotionPlanner(int degree, int num_traj_segments);

	private:
		int degree_;
		int n_traj_segments_;
		int traj_dimension_ = 2;

		drake::solvers::MathematicalProgram prog_;
		drake::symbolic::Variable t_;

		// Monomial basis of t
		symbolic_vector_t m_;
		symbolic_vector_t m_dot_;
		symbolic_vector_t m_ddot_;

		// Transformation matrices for m
		symbolic_matrix_t T_ ;
		symbolic_matrix_t T_dot_;
		symbolic_matrix_t T_ddot_;

		std::vector<symbolic_matrix_t> coeffs_;

		void construct_transform_matrix(
				symbolic_matrix_t *T, symbolic_vector_t *v
				);

		Eigen::MatrixXd get_transformation_matrix_at_t(
				double t, symbolic_vector_t v
				);
};
