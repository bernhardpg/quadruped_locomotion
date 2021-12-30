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
			std::vector<int> num_slack_vars_;

			std::vector<std::unique_ptr<
				drake::solvers::MathematicalProgram>
				> quadratic_progs_;

			// Decision variables
			std::vector<symbolic_vector_t> u_dots_;
			std::vector<symbolic_vector_t> lambdas_;
			std::vector<symbolic_vector_t> slack_vars_;

			// Original task matrices
			std::vector<Eigen::MatrixXd> A_matrs_orig_;
			std::vector<Eigen::VectorXd> b_vecs_orig_;
			std::vector<Eigen::MatrixXd> D_matrs_orig_;
			std::vector<Eigen::VectorXd> f_vecs_orig_;

			// Accumulated task matrices
			std::vector<Eigen::MatrixXd> A_matrs_accum_;
			std::vector<Eigen::VectorXd> b_vecs_accum_;
			std::vector<Eigen::MatrixXd> D_matrs_accum_;
			std::vector<Eigen::VectorXd> f_vecs_accum_;

			// Task matrices for final optimization problem
			std::vector<Eigen::MatrixXd> D_matrs_;

			// Solutions to higher order optimization problem
			std::vector<Eigen::VectorXd> x_star_;
			std::vector<Eigen::VectorXd> v_star_;
			
			// Null space matrices for accumulated tasks
			std::vector<Eigen::MatrixXd> Z_matrs_;

			void SetupQPs();
			void CreateNewMathProgForTask(int task_i);
			void CreateEmptyMathProgs(); // TODO: Rename, probably change what this does
			void CreateDecisionVariablesForTask(int index);
			void CreateSlackVariablesForTask(int index);
			void CreateSlackVariablesForTask(
					int task_i, int num_slack_variables_for_task
					);

			// Matrix creation
			void AccumulateAMatrices(); // TODO: rename
			void AccumulateBVectors();
			void AccumulateDMatrices();
			void ConstructNullSpaceMatrices();
			Eigen::MatrixXd ConstructNullSpaceMatrixFromPrevious(
					int task_i
					);
			void ConstructDMatrices();
			void ConstructDMatrix(int task_i);


			void AddIneqConstraintsForTask(int task_i);

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

			symbolic_vector_t GetAllDecisionVarsForTask(int task_i);

			// TODO: It is now time to create a math library
			Eigen::VectorXd CreateInfVector(int size);
			Eigen::MatrixXd CalcNullSpaceProjMatrix(Eigen::MatrixXd A);
			Eigen::MatrixXd CalcPseudoInverse(Eigen::MatrixXd A);
			void PrintMatrix(Eigen::MatrixXd matr);
			void PrintMatrixSize(
					std::string name, Eigen::MatrixXd matr
					);
			void PrintMatrixSize(
					std::string name, symbolic_vector_t matr
					);
	};
}
