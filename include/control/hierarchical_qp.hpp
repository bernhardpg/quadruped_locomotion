#pragma once

#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>

#include <Eigen/Core>

#include "variable_types.hpp"
#include "anymal_constants.hpp"

namespace control
{
	class HierarchicalQP
	{
		public:
			HierarchicalQP();
			~HierarchicalQP(){};

		private:
			drake::solvers::MathematicalProgram prog_;

			int	num_decision_vars_;
			symbolic_vector_t u_dot_;	
			symbolic_vector_t lambda_;	

			void AddEqConstraint(
					Eigen::MatrixXd A, Eigen::VectorXd b
					);
			void AddIneqConstraint(
					Eigen::MatrixXd D, Eigen::VectorXd f
					);

			symbolic_vector_t GetDecisionVars();
	};
}
