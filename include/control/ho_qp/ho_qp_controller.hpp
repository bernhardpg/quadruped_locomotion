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
#include "control/ho_qp/ho_qp_problem.hpp"

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

			// ******* //
			// TESTING //
			// ******* //

			void Test2Tasks(); // TODO: Rename or replace 
			void Test3Tasks(); // TODO: Rename or replace 
			Eigen::VectorXd TestLinearProgram(TaskDefinition task);
			Eigen::VectorXd TestLinearProgram(
					Eigen::MatrixXd A, Eigen::VectorXd b,
					Eigen::MatrixXd D, Eigen::VectorXd f
					);
			void CheckSolutionValid(
					Eigen::MatrixXd A, Eigen::VectorXd b,
					Eigen::MatrixXd D, Eigen::VectorXd f,
					Eigen::VectorXd sol);
	};
}

