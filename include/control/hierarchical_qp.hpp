#pragma once

#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>

#include <ros/console.h>

#include <Eigen/Core>

#include "variable_types.hpp"
#include "anymal_constants.hpp"
#include "helper_functions.hpp"
#include "math.hpp"
#include "control/optimization_task.hpp"

namespace control
{
	// USE OF NOTATION:
	// This class definition more or less follows the notation from 
	// 'Perception-less Terrain Adaptation through Whole
	// Body Control and Hierarchical Optimization', Section III

	class HierarchicalQP // TODO: rename to hierarchical optimization?
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

			void TestTasks(); // TODO: Rename or replace 

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

			Eigen::VectorXd x_solution_; // TODO: rename?
			std::vector<Eigen::VectorXd> slack_solutions_; // TODO: rename?
			std::vector<Eigen::VectorXd> slack_solutions_accum_; // TODO: rename?
			void SetSlackVarsToZero(); // TODO move?
			void GetAccumulatedSlackSolutions();

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
			std::vector<Eigen::VectorXd> f_vecs_;

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
			void AccumulateFVecs();
			void ConstructNullSpaceMatrices();
			Eigen::MatrixXd ConstructNullSpaceMatrixFromPrevious(
					int task_i
					);
			void ConstructDMatrices();
			void ConstructDMatrix(int task_i);
			void ConstructFVec(int task_i);

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

			symbolic_vector_t GetAllDecisionVarsForTask(int task_i);
	};
}
