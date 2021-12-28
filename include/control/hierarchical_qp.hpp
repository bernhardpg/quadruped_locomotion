#pragma once

#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>

#include <Eigen/Core>

#include "variable_types.hpp"
#include "anymal_constants.hpp"

namespace control
{
	const double kInf = 9999999; // TODO: move to another file

	class HierarchicalQP
	{
		public:
			HierarchicalQP();
			HierarchicalQP(int num_tasks);
			~HierarchicalQP(){};

		private:
			int num_tasks_;
			int num_contacts_;
			int	num_decision_vars_;

			std::vector<std::unique_ptr<
				drake::solvers::MathematicalProgram>
				> quadratic_progs_;
			std::vector<symbolic_vector_t> decision_variables_;
			std::vector<Eigen::MatrixXd> eq_const_matrices_;
			std::vector<Eigen::VectorXd> eq_const_vectors_;
			std::vector<Eigen::MatrixXd> ineq_const_matrices_;
			std::vector<Eigen::VectorXd> ineq_const_vectors_;
			std::vector<symbolic_vector_t> slack_variables_;

			void PopulateVariables(); // TODO: Rename or replace 
			void CreateQPWithIndex(int index);
			void CreateDecisionVariables();
			void AddEqConstraint(
					Eigen::MatrixXd A,
					Eigen::VectorXd b,
					symbolic_vector_t decision_variables
					);
			void AddIneqConstraint(
					Eigen::MatrixXd D,
					Eigen::VectorXd f,
					symbolic_vector_t decision_variables
					);

			Eigen::VectorXd CreateInfVector(int size);

			void Test();
	};
}
