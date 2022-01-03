#include <drake/solvers/mathematical_program.h>
#include <ros/console.h>
#include <assert.h>

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

	class HoQpProblem
	{
		public:
			HoQpProblem();

			void SetTask(TaskDefinition new_task);
			void SetHigherPriorityTask(TaskDefinition higher_pri_task);

			TaskDefinition GetAccumulatedTask();

		private:
			drake::solvers::MathematicalProgram prog_; 

			// TODO: Are these needed?
			symbolic_vector_t u_dot_;
			symbolic_vector_t lambda_;
			symbolic_vector_t v_;

			// TODO: are these needed?
			Eigen::MatrixXd H_;
			Eigen::MatrixXd c_;
			Eigen::MatrixXd D_;
			Eigen::MatrixXd f_;

			TaskDefinition curr_task_;
			TaskDefinition higher_pri_task_;

			bool IsHigherPriTaskDefined();

			TaskDefinition ConcatenateTasks(
					TaskDefinition &t1, TaskDefinition &t2
					);
			Eigen::MatrixXd ConcatenateMatrices(
					Eigen::MatrixXd &m1, Eigen::MatrixXd &m2
					);
			Eigen::VectorXd ConcatenateVectors(
					Eigen::VectorXd &v1, Eigen::VectorXd &v2
					);
	};
}