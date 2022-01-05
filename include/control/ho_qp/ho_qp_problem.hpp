#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>

#include <ros/console.h>
#include <assert.h>

#include "math.hpp"
#include "variable_types.hpp"
#include "helper_functions.hpp"

namespace control
{
	struct TaskDefinition
	{
		Eigen::MatrixXd A;
		Eigen::VectorXd b;
		Eigen::MatrixXd D;
		Eigen::VectorXd f;
	};

	// TODO: Remove?
	void PrintTask(TaskDefinition task)
	{
		std::cout << "A:" << std::endl;
		PrintMatrix(task.A);
		std::cout << "b:" << std::endl;
		PrintMatrix(task.b);
		std::cout << "D:" << std::endl;
		PrintMatrix(task.D);
		std::cout << "f:" << std::endl;
		PrintMatrix(task.f);
	}

	// USE OF NOTATION:
	// These definition more or less follows the notation from 
	// 'Perception-less Terrain Adaptation through Whole
	// Body Control and Hierarchical Optimization', Section III

	class HoQpProblem
	{
		public:
			HoQpProblem();
			HoQpProblem(TaskDefinition new_task);
			HoQpProblem(
					TaskDefinition new_task, HoQpProblem *higher_pri_problem_ 
					);

			// ***************** //
			// SETTERS & GETTERS //
			// ***************** //

			// TODO: Replace getters with direct variables for efficiency?
			TaskDefinition GetAccumTasks();
			Eigen::MatrixXd GetAccumD();
			Eigen::VectorXd GetAccumF();
			Eigen::MatrixXd GetAccumNullspaceMatrix();

			int GetAccumNumSlackVars();

			Eigen::VectorXd GetSolution();
			Eigen::VectorXd GetSlackSolutions();
			Eigen::VectorXd GetAccumSlackSolutions();

			symbolic_vector_t GetAllDecisionVars(); // TODO: This should have another name

		private:
			drake::solvers::MathematicalProgram prog_; 
			drake::solvers::MathematicalProgramResult result_;

			int num_slack_vars_;
			int num_decision_vars_;
			bool has_eq_constraints_;
			bool has_ineq_constraints_;
			bool is_higher_pri_problem_defined_; 

			symbolic_vector_t decision_vars_;
			symbolic_vector_t slack_vars_;

			Eigen::VectorXd decision_vars_solutions_;
			Eigen::VectorXd slack_vars_solutions_;

			Eigen::MatrixXd H_;
			Eigen::MatrixXd c_;
			Eigen::MatrixXd D_;
			Eigen::MatrixXd f_;
			
			Eigen::MatrixXd accum_Z_;

			Eigen::MatrixXd accum_Z_prev_; 
			Eigen::VectorXd accum_slack_solutions_prev_; 
			Eigen::VectorXd x_prev_;
			int num_prev_slack_vars_;

			TaskDefinition curr_task_;
			
			TaskDefinition accum_tasks_; // TODO: rename?
			TaskDefinition accum_tasks_prev_; 
			Eigen::VectorXd accum_slack_vars_; 

			HoQpProblem *higher_pri_problem_;

			// ********************* //
			// MATRIX INITIALIZATION //
			// ********************* //

			void AccumulateTasks();
			void AccumulateSlackSolutions();

			void SetPrevProblemValues();
			void InitPrevProblemValuesToDefault();
			TaskDefinition CreateEmptyTask(int num_decision_vars);
			void InitPrevProblemValuesFromPrevProblem();

			void ConstructProblemMatrices();
			void ConstructAccumNullspaceMatrix();
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

			// **************** //
			// HELPER FUNCTIONS //
			// **************** //

			void InitTaskVariables();
			TaskDefinition ConcatenateTasks(
					TaskDefinition t1, TaskDefinition t2
					);
	};
}
