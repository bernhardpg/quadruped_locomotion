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
			~HoQpController(){};

			void CalcJointCmd(
					Eigen::Matrix<double,kNumGenCoords, 1> q,
					Eigen::Matrix<double,kNumGenVels, 1> u
					);
			void SetComCmd(
					Eigen::VectorXd r_cmd,
					Eigen::VectorXd r_dot_cmd,
					Eigen::VectorXd r_ddot_cmd
					);
			void SetLegCmd(
					Eigen::VectorXd r_c_cmd,
					Eigen::VectorXd r_c_dot_cmd,
					Eigen::VectorXd r_c_ddot_cmd,
					std::vector<int> legs_in_contact
					);

			Eigen::VectorXd GetJointAccelerationCmd();
			Eigen::VectorXd GetJointTorqueCmd();

		private:
			Dynamics robot_dynamics_;

			// ********* //
			// Constants //
			// ********* //

			bool run_once_ = false;
			// TODO: Set actual torque limits and friction coeff
			const double max_torque_ = 100;
			const double min_torque_ = -max_torque_; 
			const double friction_coeff_ = 0.5; 
			const Eigen::VectorXd max_torque_vec_
				= Eigen::VectorXd::Ones(kNumJoints) * max_torque_;
			const Eigen::VectorXd min_torque_vec_
				= Eigen::VectorXd::Ones(kNumJoints) * min_torque_;

			// ********************* //
			// DYNAMICS & KINEMATICS // 
			// ********************* //

			Eigen::MatrixXd M_;
			Eigen::MatrixXd M_j_;
			Eigen::VectorXd c_;
			Eigen::VectorXd c_j_;
			Eigen::MatrixXd J_c_;
			Eigen::MatrixXd J_c_j_t_;
			Eigen::VectorXd J_c_dot_u_;
			Eigen::MatrixXd J_b_pos_;
			Eigen::MatrixXd J_b_rot_;

			void UpdateModelDynamics(
					Eigen::Matrix<double,kNumGenCoords, 1> q,
					Eigen::Matrix<double,kNumGenVels, 1> u
					);
			void CalcJointTorquesCmd();

			// ********** //
			// CONTROLLER // 
			// ********** //
			Eigen::VectorXd r_cmd_;
			Eigen::VectorXd r_dot_cmd_;
			Eigen::VectorXd r_ddot_cmd_;

			Eigen::VectorXd r_c_cmd_;
			Eigen::VectorXd r_c_dot_cmd_;
			Eigen::VectorXd r_c_ddot_cmd_;
			std::vector<int> legs_in_contact_;
			
			int num_contacts_ = 0;
			int	num_decision_vars_ = 0;

			Eigen::VectorXd solution_;
			Eigen::VectorXd tau_cmd_;
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
			TaskDefinition ConstructNoContactMotionTask();
			TaskDefinition ConstructComPosTrajTask(
					const Eigen::VectorXd &q, const Eigen::VectorXd &u 
					);
			TaskDefinition ConstructComRotTrajTask(
					const Eigen::VectorXd &q, const Eigen::VectorXd &u 
					);
			TaskDefinition ConstructForceMinimizationTask();
			TaskDefinition ConstructJointAccMinimizationTask();

			// **************** //
			// HELPER FUNCTIONS //
			// **************** //

			template <typename Derived>
			Derived GetFloatingBaseRows(
					const Eigen::DenseBase<Derived> &m
					);
			template <typename Derived>
			Derived GetJointRows(
					const Eigen::DenseBase<Derived> &m
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

