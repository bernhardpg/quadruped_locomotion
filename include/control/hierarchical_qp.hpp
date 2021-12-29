#pragma once

#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>

#include <ros/console.h>

#include <Eigen/Core>

#include "variable_types.hpp"
#include "anymal_constants.hpp"

namespace control
{
	// USE OF NOTATION:
	// This class definition more or less follows the notation from 
	// 'Perception-less Terrain Adaptation through Whole
	// Body Control and Hierarchical Optimization', Section III

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

			// ******* //
			// TESTING //
			// ******* //

			void PopulateTestVariables(); // TODO: Rename or replace 

			// ******************** //
			// OPTIMIZATION PROBLEM //
			// ******************** //

			int	num_decision_vars_;

			std::vector<std::unique_ptr<
				drake::solvers::MathematicalProgram>
				> quadratic_progs_;

			// Decision variables
			std::vector<symbolic_vector_t> u_dots_;
			std::vector<symbolic_vector_t> lambdas_;

			// Original task matrices
			std::vector<Eigen::MatrixXd> task_eq_const_As_;
			std::vector<Eigen::VectorXd> task_eq_const_bs_;
			std::vector<Eigen::MatrixXd> task_ineq_const_Ds_;
			std::vector<Eigen::VectorXd> task_ineq_const_fs_;
			std::vector<symbolic_vector_t> task_ineq_slack_variables_;

			// Matrices reformulated to optimization problem form
			std::vector<Eigen::MatrixXd> A_matrices_;
			std::vector<Eigen::VectorXd> b_vectors_;
			std::vector<Eigen::MatrixXd> D_matrices_;
			std::vector<Eigen::VectorXd> f_vectors_;

			std::vector<Eigen::MatrixXd> Z_matrices_;

			void SetupQPs();
			void CreateNewMathProgForTask(int task_i);
			void CreateEmptyMathProgs(); // TODO: Rename, probably change what this does
			void CreateDecisionVariablesForTask(int index);
			void CreateSlackVariablesForTask(int index);

			// Matrix creation
			void AccumulateAMatrices();
			void AccumulateBVectors();
			void AccumulateDMatrices();
			void ConstructNullSpaceMatrices();
			Eigen::MatrixXd ConstructNullSpaceMatrixFromPrevious(
					int task_i
					);
			void ConstructDMatrices();
			void ConstructDMatrix(int task_i);

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

			// **************** //
			// HELPER FUNCTIONS //
			// **************** //

			// TODO: It is now time to create a math library
			Eigen::VectorXd CreateInfVector(int size);
			Eigen::MatrixXd CalcNullSpaceProjMatrix(Eigen::MatrixXd A);
			Eigen::MatrixXd CalcPseudoInverse(Eigen::MatrixXd A);
			void PrintMatrix(Eigen::MatrixXd matr);
			void PrintMatrixSize(
					std::string name, Eigen::MatrixXd matr
					);
	};
}
