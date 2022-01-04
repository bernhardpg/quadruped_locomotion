#include <drake/solvers/mathematical_program.h>
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
			Eigen::MatrixXd GetAccumA();
			Eigen::MatrixXd GetAccumD();
			Eigen::VectorXd GetAccumB();
			Eigen::VectorXd GetAccumF();
			Eigen::MatrixXd GetAccumNullspaceMatrix();

			int GetPrevAccumNumSlackVars();
			int GetAccumNumSlackVars();

			Eigen::VectorXd GetSolution();
			Eigen::VectorXd GetSlackSolutions();
			Eigen::VectorXd GetAccumSlackSolutions();

			symbolic_vector_t GetAllDecisionVars(); // TODO: This should have another name

		private:
			drake::solvers::MathematicalProgram prog_; 

			int num_slack_vars_;
			int num_decision_vars_;

			symbolic_vector_t decision_vars_;
			symbolic_vector_t slack_vars_;

			Eigen::MatrixXd H_;
			Eigen::MatrixXd c_;
			Eigen::MatrixXd D_;
			Eigen::MatrixXd f_;
			
			Eigen::MatrixXd accum_Z_;
			Eigen::MatrixXd accum_Z_prev_; 

			TaskDefinition curr_task_;
			
			TaskDefinition accum_tasks_;
			Eigen::VectorXd accum_slack_vars_; 

			HoQpProblem *higher_pri_problem_;

			// ********************* //
			// MATRIX INITIALIZATION //
			// ********************* //

			void AccumulateTasks();
			void AccumulateSlackSolutions();
			void SetAccumNullspacePrev();

			void ConstructAccumNullspaceMatrix();
			void ConstructDMatrix();
			void ConstructFVector();
			void ConstructHMatrix();
			void ConstructCVector();

			// ******************** //
			// OPTIMIZATION PROBLEM //
			// ******************** //

			void CreateDecisionVars();
			void CreateSlackVars();
			void AddIneqConstraints();
			void AddQuadraticCost();

			// **************** //
			// HELPER FUNCTIONS //
			// **************** //

			bool IsHigherPriProblemDefined();
			TaskDefinition ConcatenateTasks(
					TaskDefinition t1, TaskDefinition t2
					);
			Eigen::MatrixXd ConcatenateMatrices(
					Eigen::MatrixXd m1, Eigen::MatrixXd m2
					);
			Eigen::VectorXd ConcatenateVectors(
					Eigen::VectorXd v1, Eigen::VectorXd v2
					);
	};
}
