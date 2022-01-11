#pragma once

#include "dynamics/dynamics.hpp"
#include "anymal_constants.hpp"
#include "helper_functions.hpp"
#include "math.hpp"
#include "control/ho_qp/ho_qp_problem.hpp"
#include "control/ho_qp/task_definition.hpp"
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

			void Update(
					Eigen::Matrix<double,kNumGenCoords, 1> q,
					Eigen::Matrix<double,kNumGenVels, 1> u
					);

		private:
			const double max_torque_ = 100; // TODO: Set the actual torque limit?
			const double min_torque_ = -max_torque_; 
			const double friction_coeff_ = 0.5;  // TODO: set more accurately?
			int num_tasks_;
			int num_contacts_;
			int	num_decision_vars_;

			bool run_once_ = false; // TODO: Only for dev
			Dynamics robot_dynamics_; // TODO: Should this own its own dynamics object?

			Eigen::MatrixXd mass_matrix_;
			Eigen::MatrixXd bias_vector_;
			Eigen::MatrixXd contact_jacobian_;
			Eigen::MatrixXd contact_jacobian_dot_;
			Eigen::MatrixXd body_jacobian_pos_;
			Eigen::MatrixXd body_jacobian_rot_;

			TaskDefinition ConstructFloatingBaseEomTask();
			TaskDefinition ConstructJointTorqueTask();
			TaskDefinition ConstructFrictionConeTask();
			TaskDefinition ConstructNoContactMotionTask(
					Eigen::VectorXd u
					);
			TaskDefinition ConstructComPosTrajTask(
					Eigen::VectorXd com_vel 
					);
			TaskDefinition ConstructComRotTrajTask(
					Eigen::VectorXd ang_vel // TODO: rename 
					);

			Eigen::MatrixXd GetFloatingBaseRows(Eigen::MatrixXd &m);
			Eigen::MatrixXd GetJointRows(Eigen::MatrixXd &m);

			void UpdateDynamicsTerms(
					Eigen::Matrix<double,kNumGenCoords, 1> q,
					Eigen::Matrix<double,kNumGenVels, 1> u
					);


			// ******* //
			// TESTING //
			// ******* //

			void TestEomConstraint();
			void TestTwoTasksEqFirst(); 
			void TestTwoTasksIneqFirst();
			void TestThreeTasks();
			void TestFourTasks();
			void TestSingleEqTask();

			Eigen::VectorXd SolveWithLinearProgram(TaskDefinition task);
			Eigen::VectorXd SolveWithLinearProgram(
					Eigen::MatrixXd A, Eigen::VectorXd b,
					Eigen::MatrixXd D, Eigen::VectorXd f
					);
	};
}

