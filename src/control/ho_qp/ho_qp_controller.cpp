#include "control/ho_qp/ho_qp_controller.hpp"

namespace control
{
	HoQpController::HoQpController()
	{
		num_contacts_ = 4; // TODO: generalize this
		num_decision_vars_ = kNumGenVels + kNumPosDims * num_contacts_;
	}

	void HoQpController::Update(
			Eigen::Matrix<double,kNumGenCoords, 1> q,
			Eigen::Matrix<double,kNumGenVels, 1> u
			)
	{
		if (run_once_ == false)
		{
			UpdateModelDynamics(q,u);
			std::vector<TaskDefinition> tasks = ConstructTasks(q,u);
			std::vector<std::shared_ptr<HoQpProblem>>
				opt_problems = ConstructOptProblems(tasks);

			Eigen::VectorXd sol = opt_problems.back()->GetSolution();
			q_j_ddot_cmd_ = sol.block<kNumJoints,1>(kNumTwistCoords,0);
		}
	}

	Eigen::VectorXd HoQpController::GetJointAccelerationCmd()
	{
		return q_j_ddot_cmd_;
	}

	// ********************* //
	// DYNAMICS & KINEMATICS // 
	// ********************* //

	void HoQpController::UpdateModelDynamics(
			Eigen::Matrix<double,kNumGenCoords, 1> q,
			Eigen::Matrix<double,kNumGenVels, 1> u
			)
	{
		robot_dynamics_.SetState(q,u);
		// TODO: Assumes 4 contact points!
		M_ = robot_dynamics_.GetMassMatrix();
		c_ = robot_dynamics_.GetBiasVector();
		J_c_ = robot_dynamics_.GetStackedContactJacobianInW();
		J_c_dot_u_ = robot_dynamics_.GetStackedContactAccInW();

		auto J_b = robot_dynamics_.GetBaseJacobianInW();
		J_b_rot_ = J_b.block(0,0,3,kNumGenVels);
		J_b_pos_ = J_b.block(3,0,3,kNumGenVels);
	}

	// ********** //
	// CONTROLLER // 
	// ********** //

	std::vector<std::shared_ptr<HoQpProblem>>
		HoQpController::ConstructOptProblems(
			std::vector<TaskDefinition> &tasks
			)
	{
		std::vector<std::shared_ptr<HoQpProblem>>
			opt_problems(tasks.size());

		opt_problems[0] =
			std::shared_ptr<HoQpProblem>(new HoQpProblem(tasks[0]));
		for (int task_i = 1; task_i < tasks.size(); ++task_i)
		{
			opt_problems[task_i] =
				std::shared_ptr<HoQpProblem>(
						new HoQpProblem(
							tasks[task_i], opt_problems[task_i - 1]
							)
						);
		}

		return opt_problems;
	}

	// ***************** //
	// TASK CONSTRUCTION //
	// ***************** //

	std::vector<TaskDefinition> HoQpController::ConstructTasks(
			Eigen::VectorXd q, Eigen::VectorXd u
			)
	{
		TaskDefinition fb_eom_task = ConstructFloatingBaseEomTask();

		TaskDefinition joint_torque_task = ConstructJointTorqueTask();
		TaskDefinition friction_cone_task = ConstructFrictionConeTask();
		TaskDefinition joint_torque_and_friction_task =
			ConcatenateTasks(joint_torque_task, friction_cone_task);

		TaskDefinition no_contact_motion_task =
			ConstructNoContactMotionTask();

		auto v_IB_I = u.block(0,0,3,1);
		TaskDefinition com_pos_traj_task =
			ConstructComPosTrajTask(v_IB_I);
		auto w_IB = u.block(kNumPosDims,0,3,1);
		TaskDefinition com_rot_traj_task =
			ConstructComRotTrajTask(w_IB);
		TaskDefinition com_traj_task = 
			ConcatenateTasks(com_pos_traj_task, com_rot_traj_task);

		TaskDefinition force_min_task = ConstructForceMinimizationTask();
		TaskDefinition acc_min_task = ConstructJointAccMinimizationTask();

		std::vector<TaskDefinition> tasks{
			fb_eom_task,
			joint_torque_and_friction_task,
			no_contact_motion_task,
			com_traj_task,
			force_min_task,
		};

		return tasks;
	}

	TaskDefinition HoQpController::ConstructComPosTrajTask(
			Eigen::VectorXd v_IB_I 
			)
	{
		Eigen::VectorXd vd_IB_I(3);
		vd_IB_I << -0.2, 0, 0; // TODO: Take from traj generator

		double k_pos = 1.0;
		double k_vel = 1.0;

		Eigen::MatrixXd zero = 
			Eigen::MatrixXd::Zero(
					J_b_pos_.rows(), kNumPosDims * num_contacts_
					);

		Eigen::MatrixXd A(J_b_pos_.rows(), num_decision_vars_);
		A << J_b_pos_, zero;

		Eigen::VectorXd b(J_b_pos_.rows());
		b << k_vel * (vd_IB_I - v_IB_I); // TODO: missing a bunch of terms

		TaskDefinition com_pos_traj_task = {.A=A, .b=b};
		return com_pos_traj_task;
	}

	TaskDefinition HoQpController::ConstructComRotTrajTask(
			Eigen::VectorXd w_IB 
			)
	{
		Eigen::VectorXd wd_IB(3);
		wd_IB << 0, 0, 0;

		double k_pos = 1.0;
		double k_vel = 0.1;

		Eigen::MatrixXd zero = 
			Eigen::MatrixXd::Zero(
					J_b_rot_.rows(), kNumPosDims * num_contacts_
					);

		Eigen::MatrixXd A(J_b_rot_.rows(), num_decision_vars_);
		A << J_b_rot_, zero;

		Eigen::VectorXd b(J_b_pos_.rows());
		b << k_vel * (wd_IB - w_IB); // TODO: missing a bunch of terms here

		TaskDefinition com_pos_traj_task = {.A=A, .b=b};
		return com_pos_traj_task;
	}

	TaskDefinition HoQpController::ConstructNoContactMotionTask()
	{
		Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(
				kNumPosDims * num_contacts_, kNumPosDims * num_contacts_);

		Eigen::MatrixXd A(kNumPosDims * num_contacts_, num_decision_vars_);
		A << J_c_, zero;

		Eigen::VectorXd b(kNumPosDims * num_contacts_);
		b << -J_c_dot_u_;

		TaskDefinition no_contact_motion_task = {.A=A, .b=b};
		return no_contact_motion_task;
	}

	TaskDefinition HoQpController::ConstructFrictionConeTask()
	{
		const int num_constraints_per_leg = 4;
		Eigen::MatrixXd friction_pyramic_constraint(
				num_constraints_per_leg, kNumPosDims
				);
		// Corresponds to
		// abs(lambda_x) <= coeff * lambda_z
		// abs(lambda_y) <= coeff * lambda_z
		friction_pyramic_constraint <<
			1, 0, -friction_coeff_,
			-1, 0, -friction_coeff_,
			0, 1, -friction_coeff_,
			0, -1, -friction_coeff_;

		// TODO: Generalize this to when not all feet are in contact
		Eigen::MatrixXd D(
				num_constraints_per_leg * num_contacts_, num_decision_vars_
				);
		D.setZero();
		for (int i = 0; i < num_contacts_; ++i)
		{
			D.block<num_constraints_per_leg,kNumPosDims>
				(i * num_constraints_per_leg,
				 kNumTwistCoords + i * kNumPosDims)
				= friction_pyramic_constraint;
		}
		Eigen::VectorXd f = Eigen::VectorXd::Zero(D.rows());

		TaskDefinition friction_cone_task = {.D=D, .f=f};
		return friction_cone_task;
	}

	TaskDefinition HoQpController::ConstructJointTorqueTask()
	{
		Eigen::MatrixXd M_j = GetJointRows(M_);
		Eigen::VectorXd c_j = GetJointRows(c_);
		Eigen::MatrixXd J_c_t = J_c_.transpose();
		Eigen::MatrixXd J_c_j_t = GetJointRows(J_c_t);

		Eigen::MatrixXd D(kNumJoints,num_decision_vars_);
		D << M_j, -J_c_j_t; 

		Eigen::VectorXd f_max(kNumJoints);
		f_max << max_torque_vec_ - c_j;

		Eigen::VectorXd f_min(kNumJoints);
		f_min << min_torque_vec_ - c_j;

		// Positive and negative limit
		Eigen::MatrixXd D_tot = ConcatenateMatrices(D,-D);
		Eigen::VectorXd f_tot = ConcatenateVectors(f_max,-f_min);

		TaskDefinition joint_torque_constraint = {.D=D_tot, .f=f_tot};
		return joint_torque_constraint;
	}

	TaskDefinition HoQpController::ConstructFloatingBaseEomTask()
	{
		Eigen::MatrixXd M_fb = GetFloatingBaseRows(M_);
		Eigen::VectorXd c_fb = GetFloatingBaseRows(c_);
		Eigen::MatrixXd J_c_t = J_c_.transpose();
		Eigen::MatrixXd J_c_fb_t = GetFloatingBaseRows(J_c_t);

		Eigen::MatrixXd A(kNumTwistCoords,num_decision_vars_);
		A << M_fb, -J_c_fb_t; 

		Eigen::VectorXd b(kNumTwistCoords);
		b << -c_fb;

		TaskDefinition fb_eom_task = {.A=A, .b=b};
		return fb_eom_task;
	}

	TaskDefinition HoQpController::ConstructJointAccMinimizationTask()
	{
		Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(
				kNumGenVels, kNumGenVels
				);
		Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(
				kNumGenVels, kNumPosDims * num_contacts_
				);
		Eigen::MatrixXd A(kNumGenVels, num_decision_vars_);
		A << eye, zero;

		Eigen::VectorXd b = Eigen::VectorXd::Zero(A.rows());

		TaskDefinition acc_min_task = {.A=A, .b=b};
		return acc_min_task;
	}

	TaskDefinition HoQpController::ConstructForceMinimizationTask()
	{
		Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(
				kNumPosDims * num_contacts_, kNumGenVels 
				);
		Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(
				kNumPosDims * num_contacts_, kNumPosDims * num_contacts_
				);
		Eigen::MatrixXd A(kNumPosDims * num_contacts_, num_decision_vars_);
		A << zero, eye;

		Eigen::VectorXd b = Eigen::VectorXd::Zero(A.rows());

		TaskDefinition force_min_task = {.A=A, .b=b};
		return force_min_task;
	}

	// **************** //
	// HELPER FUNCTIONS //
	// **************** //

	Eigen::VectorXd HoQpController::GetFloatingBaseRows(
			Eigen::VectorXd &v
			)
	{
		Eigen::VectorXd v_fb
			= v.block(0,0,kNumTwistCoords,v.cols());
		return v_fb;
	}

	Eigen::MatrixXd HoQpController::GetFloatingBaseRows(
			Eigen::MatrixXd &m
			)
	{
		Eigen::MatrixXd m_fb
			= m.block(0,0,kNumTwistCoords,m.cols());
		return m_fb;
	}

	// TODO: Use Eigenderived instead of separate functions
	Eigen::VectorXd HoQpController::GetJointRows(
			Eigen::VectorXd &v
			)
	{
		Eigen::VectorXd v_j
			= v.block(kNumTwistCoords,0,kNumJoints,1);
		return v_j;
	}

	Eigen::MatrixXd HoQpController::GetJointRows(
			Eigen::MatrixXd &m
			)
	{
		Eigen::MatrixXd m_j
			= m.block(kNumTwistCoords,0,kNumJoints,m.cols());
		return m_j;
	}
}
