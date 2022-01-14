#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>

#include <ros/console.h>
#include <assert.h>

#include "math.hpp"
#include "variable_types.hpp"
#include "helper_functions.hpp"
#include "control/ho_qp/task_definition.hpp"

namespace control
{
	// USE OF NOTATION:
	// These definition more or less follows the notation from 
	// 'Perception-less Terrain Adaptation through Whole
	// Body Control and Hierarchical Optimization', Section III

	class HoQpProblem
	{
		public:
			HoQpProblem(TaskDefinition &new_task);
			HoQpProblem(
					TaskDefinition &new_task,
					std::shared_ptr<HoQpProblem> higher_pri_problem
					);

			// ***************** //
			// SETTERS & GETTERS //
			// ***************** //

			// TODO: Replace getters with direct variables for efficiency?
			TaskDefinition GetStackedTasks();
			Eigen::MatrixXd GetStackedNullspaceMatrix();

			int GetStackedNumSlackVars();

			Eigen::VectorXd GetSolution();
			Eigen::VectorXd GetSlackSolutions();
			Eigen::VectorXd GetStackedSlackSolutions();

			void SetAllDecisionVars(); // TODO rename

		private:
			drake::solvers::MathematicalProgram prog_; 
			drake::solvers::MathematicalProgramResult result_;

			int num_slack_vars_;
			int num_decision_vars_;
			bool has_eq_constraints_;
			bool has_ineq_constraints_;
			bool is_higher_pri_problem_defined_; 

			// Convenience matrices that are used multiple times
			const double eps_ = 1e-15;
			Eigen::MatrixXd eps_matrix_; 
			Eigen::MatrixXd eye_nv_nv_;
			Eigen::MatrixXd zero_nv_nx_;
			Eigen::MatrixXd A_curr_Z_prev_;

			bool print_solver_details_ = false; // TODO: only for debugging

			variable_vector_t decision_vars_;
			variable_vector_t slack_vars_;
			variable_vector_t all_decision_vars_;

			Eigen::VectorXd decision_vars_solutions_;
			Eigen::VectorXd slack_vars_solutions_;

			Eigen::MatrixXd H_;
			Eigen::MatrixXd c_;
			Eigen::MatrixXd D_;
			Eigen::MatrixXd f_;
			
			Eigen::MatrixXd stacked_Z_;

			Eigen::MatrixXd stacked_Z_prev_; 
			Eigen::VectorXd stacked_slack_solutions_prev_; 
			Eigen::VectorXd x_prev_;
			int num_prev_slack_vars_;

			TaskDefinition curr_task_;
			
			TaskDefinition stacked_tasks_; // TODO: rename?
			TaskDefinition stacked_tasks_prev_; 
			Eigen::VectorXd stacked_slack_vars_; 

			std::shared_ptr<HoQpProblem> higher_pri_problem_;

			// ********************* //
			// MATRIX INITIALIZATION //
			// ********************* //

			void InitTaskVariables();
			void StackTasks();
			void StackSlackSolutions();

			void LoadPrevProblemData();
			void InitPrevProblemValuesToDefault();
			TaskDefinition CreateEmptyTask(int num_decision_vars);
			void InitPrevProblemValuesFromPrevProblem();

			void ConstructProblemMatrices();
			void ConstructStackedNullspaceMatrix();
			void ConstructNullspaceMatrixFromPrev();

			void ConstructDMatrix();
			void ConstructDMatrixWithCurrTask();

			void ConstructFVector();
			void ConstructHMatrix();
			void ConstructCVector();

			// ******************** //
			// OPTIMIZATION PROBLEM //
			// ******************** //

			void FormulateOptimizationProblem();
			void CreateDecisionVars();
			void CreateSlackVars();
			void AddIneqConstraints();
			void AddQuadraticCost();
			void SolveQp();

	};
}
