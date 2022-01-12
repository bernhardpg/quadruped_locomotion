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
			Eigen::VectorXd GetJointAccelerationCmd();

		private:
			Dynamics robot_dynamics_; // TODO: Should this controller own its own dynamics object?

			// ********* //
			// Constants //
			// ********* //

			// TODO: Set actual torque limits and friction coeff?
			const double max_torque_ = 100;
			const double min_torque_ = -max_torque_; 
			const double friction_coeff_ = 0.5; 
			const Eigen::VectorXd max_torque_vec_;
			const Eigen::VectorXd min_torque_vec_;

			// ********************* //
			// DYNAMICS & KINEMATICS // 
			// ********************* //

			Eigen::MatrixXd M_;
			Eigen::VectorXd c_;
			Eigen::MatrixXd J_c_;
			Eigen::VectorXd J_c_dot_u_;
			Eigen::MatrixXd J_b_pos_;
			Eigen::MatrixXd J_b_rot_;

			void UpdateDynamicsTerms(
					Eigen::Matrix<double,kNumGenCoords, 1> q,
					Eigen::Matrix<double,kNumGenVels, 1> u
					);

			// ********** //
			// CONTROLLER // 
			// ********** //

			int num_tasks_;
			int num_contacts_;
			int	num_decision_vars_;

			Eigen::VectorXd q_j_ddot_cmd_;

			std::vector<std::shared_ptr<HoQpProblem>>
			ConstructOptProblems(
					std::vector<TaskDefinition> &tasks
					);

			// ***************** //
			// TASK CONSTRUCTION //
			// ***************** //

			std::vector<TaskDefinition> ConstructTasks(
					Eigen::VectorXd q, Eigen::VectorXd u
					);
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
			TaskDefinition ConstructForceMinimizationTask();
			TaskDefinition ConstructJointAccMinimizationTask();

			// **************** //
			// HELPER FUNCTIONS //
			// **************** //

			Eigen::MatrixXd GetFloatingBaseRows(Eigen::MatrixXd &m);
			Eigen::MatrixXd GetJointRows(Eigen::MatrixXd &m);



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

