#pragma once

#include "dynamics/dynamics.hpp"
#include "anymal_constants.hpp"
#include "helper_functions.hpp"
#include "math.hpp"
#include "control/ho_qp/ho_qp_problem.hpp"
#include "variable_types.hpp"

#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>

#include <ros/console.h>

#include <Eigen/Core>

namespace control
{
	class HoQpController
	{
		public:
			HoQpController();
			HoQpController(int num_tasks);
			~HoQpController(){};

		private:
			int num_tasks_;
			int num_contacts_;
			int	num_decision_vars_;

			void EquationsOfMotionConstraint();

			Dynamics robot_dynamics_; // TODO: Should this own its own dynamics object?

			// ******* //
			// TESTING //
			// ******* //

			void TestTwoTasksEqFirst(); 
			void TestTwoTasksIneqFirst();
			void TestThreeTasks();
			void TestFourTasks();

			Eigen::VectorXd SolveWithLinearProgram(TaskDefinition task);
			Eigen::VectorXd SolveWithLinearProgram(
					Eigen::MatrixXd A, Eigen::VectorXd b,
					Eigen::MatrixXd D, Eigen::VectorXd f
					);
	};
}

