#include "control/ho_qp/ho_qp_controller.hpp"

namespace control
{
	HoQpController::HoQpController() {}

	void HoQpController::CalcJointCmd(
			Eigen::Matrix<double,kNumGenCoords, 1> q,
			Eigen::Matrix<double,kNumGenVels, 1> u
			)
	{
		UpdateModelDynamics(q,u);
		std::vector<TaskDefinition> tasks = ConstructTasks(q,u);
		std::vector<std::shared_ptr<HoQpProblem>>
			opt_problems = ConstructOptProblems(tasks);

		solution_ = opt_problems.back()->GetSolution();
		CheckSolutionValid("fb_eom", tasks[0], solution_);
		CheckSolutionValid("friction and torque", tasks[1], solution_);
		CheckSolutionValid("no contact motion", tasks[2], solution_);
		CheckSolutionValid("traj tracking", tasks[3], solution_);
		CheckSolutionValid("min forces", tasks[4], solution_);
		q_j_ddot_cmd_ = solution_.block<kNumJoints,1>(kNumTwistCoords,0);
		CalcJointTorquesCmd();
	}

	void HoQpController::SetBaseCmd(
			Eigen::VectorXd base_cmd,
			Eigen::VectorXd base_dot_cmd,
			Eigen::VectorXd base_ddot_cmd
			)
	{
		base_cmd_ = base_cmd; 
		base_dot_cmd_ = base_dot_cmd;
		base_ddot_cmd_ = base_ddot_cmd;
	}

	void HoQpController::SetLegCmd(
			Eigen::VectorXd leg_cmd,
			Eigen::VectorXd leg_dot_cmd,
			Eigen::VectorXd leg_ddot_cmd,
			Eigen::VectorXd contact_pattern_cmd
			)
	{
		contact_legs_ = GetContactLegs(contact_pattern_cmd);
		num_contacts_ = contact_legs_.size();

		swing_legs_ = GetSwingLegs(contact_pattern_cmd);
		num_swing_legs_ = swing_legs_.size();

		swing_leg_cmd_ = GetSwingLegCmd(
				leg_cmd, swing_legs_, num_swing_legs_
				);
		swing_leg_dot_cmd_ = GetSwingLegCmd(
				leg_dot_cmd, swing_legs_, num_swing_legs_
				);
		swing_leg_ddot_cmd_ = GetSwingLegCmd(
				leg_ddot_cmd, swing_legs_, num_swing_legs_
				);

		// TODO remove
		assert(swing_leg_cmd_.rows() == k3D * num_swing_legs_);

		num_decision_vars_ = kNumGenVels + k3D * num_contacts_;
	}

	void HoQpController::CalcJointTorquesCmd()
	{
		Eigen::MatrixXd temp(kNumJoints,num_decision_vars_);
		temp << M_j_, -J_c_j_t_;

		tau_cmd_ = temp * solution_ + c_j_;
	}

	Eigen::VectorXd HoQpController::GetJointTorqueCmd()
	{
		return tau_cmd_;
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

		M_ = robot_dynamics_.GetMassMatrix();
		M_j_ = GetJointRows(M_);

		c_ = robot_dynamics_.GetBiasVector();
		c_j_ = GetJointRows(c_);

		J_c_ = robot_dynamics_.GetStackedContactJacobianInW(contact_legs_);
		J_c_dot_u_ = robot_dynamics_.GetStackedContactAccInW(contact_legs_);

		J_swing_ =
			robot_dynamics_.GetStackedContactJacobianInW(swing_legs_);

		Eigen::MatrixXd J_c_t = J_c_.transpose();
		J_c_j_t_ = GetJointRows(J_c_t);

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

		TaskDefinition base_pos_traj_task =
			ConstructBasePosTrajTask(q,u);
		TaskDefinition base_rot_traj_task =
			ConstructBaseRotTrajTask(q,u);
		TaskDefinition swing_leg_task =
			ConstructSwingLegTask(q,u);

		std::vector<TaskDefinition> tracking_tasks = {
			base_pos_traj_task, base_rot_traj_task, swing_leg_task
		};
		TaskDefinition tracking_task = ConcatenateTasks(tracking_tasks);

		TaskDefinition force_min_task = ConstructForceMinimizationTask();
		//TaskDefinition acc_min_task = ConstructJointAccMinimizationTask();

		std::vector<TaskDefinition> tasks{
			fb_eom_task,
			joint_torque_and_friction_task,
			no_contact_motion_task,
			tracking_task,
			force_min_task
		};

		return tasks;
	}

	TaskDefinition HoQpController::ConstructSwingLegTask(
			const Eigen::VectorXd &q, const Eigen::VectorXd &u 
			)
	{
		// TODO: move to member variables
		const double k_p = 1.0;
		const double k_v = 1.0;

		Eigen::VectorXd swing_leg_pos =
			robot_dynamics_.GetStackedFootPosInW(swing_legs_);

		Eigen::VectorXd swing_leg_vel = J_swing_ * u;

		Eigen::VectorXd cmd = swing_leg_ddot_cmd_
			+ k_p * (swing_leg_cmd_ - swing_leg_pos)
			+ k_v * (swing_leg_dot_cmd_ - swing_leg_vel);

		Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(
					J_swing_.rows(), k3D * num_contacts_
					);

		Eigen::MatrixXd A(J_swing_.rows(), num_decision_vars_);
		A << J_swing_, zero;

		Eigen::VectorXd b(J_swing_.rows());
		b << cmd;

		TaskDefinition swing_leg_task = {.A=A, .b=b};
		return swing_leg_task;
	}

	TaskDefinition HoQpController::ConstructBasePosTrajTask(
			const Eigen::VectorXd &q, const Eigen::VectorXd &u 
			)
	{
		const Eigen::VectorXd base_pos = q.block(kQuatSize,0,k3D,1);
		const Eigen::VectorXd base_vel = u.block(k3D,0,k3D,1);

		const double k_pos = 1.0;
		const double k_vel = 1.0;

		Eigen::MatrixXd zero = 
			Eigen::MatrixXd::Zero(
					J_b_pos_.rows(), k3D * num_contacts_
					);

		Eigen::MatrixXd A(J_b_pos_.rows(), num_decision_vars_);
		A << J_b_pos_, zero;

		const Eigen::VectorXd cmd = base_ddot_cmd_
			+ k_vel * (base_dot_cmd_ - base_vel);
			+ k_pos * (base_cmd_ - base_pos);

		Eigen::VectorXd b(J_b_pos_.rows());
		b << cmd;

		std::cout << "base error:\n";
		PrintMatrix(b.transpose());

		TaskDefinition base_pos_traj_task = {.A=A, .b=b};
		return base_pos_traj_task;
	}

	TaskDefinition HoQpController::ConstructBaseRotTrajTask(
			const Eigen::VectorXd &q, const Eigen::VectorXd &u 
			)
	{
		Eigen::VectorXd q_IB = q.block(0,0,kQuatSize,1);
		Eigen::VectorXd w_IB = u.block(0,0,k3D,1);

		Eigen::VectorXd wd_IB(3);
		wd_IB << 0, 0, 0; // Always try to keep no angular motion

		const double k_pos = 1.0;
		const double k_vel = 1.0;

		Eigen::MatrixXd zero = 
			Eigen::MatrixXd::Zero(
					J_b_rot_.rows(), k3D * num_contacts_
					);

		Eigen::MatrixXd A(J_b_rot_.rows(), num_decision_vars_);
		A << J_b_rot_, zero;

		const Eigen::VectorXd cmd =
			k_vel * (wd_IB - w_IB);  // TODO: implement quaternion error here

		Eigen::VectorXd b(J_b_pos_.rows());
		b << cmd;
			
		std::cout << "base rotation error:\n";
		PrintMatrix(b.transpose());

		TaskDefinition base_rot_traj_task = {.A=A, .b=b};
		return base_rot_traj_task;
	}

	TaskDefinition HoQpController::ConstructNoContactMotionTask()
	{
		Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(
				k3D * num_contacts_, k3D * num_contacts_
				);

		Eigen::MatrixXd A(k3D * num_contacts_, num_decision_vars_);
		A << J_c_, zero;

		Eigen::VectorXd b(k3D * num_contacts_);
		b << -J_c_dot_u_;

		TaskDefinition no_contact_motion_task = {.A=A, .b=b};
		return no_contact_motion_task;
	}

	TaskDefinition HoQpController::ConstructFrictionConeTask()
	{
		const int num_constraints_per_leg = 4;
		Eigen::MatrixXd friction_pyramic_constraint(
				num_constraints_per_leg, k3D
				);

		// Corresponds to
		// abs(lambda_x) <= coeff * lambda_z
		// abs(lambda_y) <= coeff * lambda_z
		friction_pyramic_constraint <<
			1, 0, -friction_coeff_,
			-1, 0, -friction_coeff_,
			0, 1, -friction_coeff_,
			0, -1, -friction_coeff_;

		Eigen::MatrixXd D(
				num_constraints_per_leg * num_contacts_, num_decision_vars_
				);
		D.setZero();
		for (int i = 0; i < num_contacts_; ++i)
		{
			D.block<num_constraints_per_leg,k3D>
				(i * num_constraints_per_leg,
				 kNumTwistCoords + contact_legs_[i] * k3D)
				= friction_pyramic_constraint;
		}
		Eigen::VectorXd f = Eigen::VectorXd::Zero(D.rows());

		TaskDefinition friction_cone_task = {.D=D, .f=f};
		return friction_cone_task;
	}

	TaskDefinition HoQpController::ConstructJointTorqueTask()
	{
		Eigen::MatrixXd D(kNumJoints,num_decision_vars_);
		D << M_j_, -J_c_j_t_; 

		Eigen::VectorXd f_max(kNumJoints);
		f_max << max_torque_vec_ - c_j_;

		Eigen::VectorXd f_min(kNumJoints);
		f_min << min_torque_vec_ - c_j_;

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
				kNumGenVels, k3D * num_contacts_
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
				k3D * num_contacts_, kNumGenVels 
				);
		Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(
				k3D * num_contacts_, k3D * num_contacts_
				);
		Eigen::MatrixXd A(k3D * num_contacts_, num_decision_vars_);
		A << zero, eye;

		Eigen::VectorXd b = Eigen::VectorXd::Zero(A.rows());

		TaskDefinition force_min_task = {.A=A, .b=b};
		return force_min_task;
	}

	// **************** //
	// HELPER FUNCTIONS //
	// **************** //

	template <typename Derived>
	Derived HoQpController::GetFloatingBaseRows(
			const Eigen::DenseBase<Derived> &m
			)
	{
		Derived m_fb
			= m.block(0,0,kNumTwistCoords,m.cols());
		return m_fb;
	}

	template <typename Derived>
	Derived HoQpController::GetJointRows(
			const Eigen::DenseBase<Derived> &m
			)
	{
		Derived m_j
			= m.block(kNumTwistCoords,0,kNumJoints,m.cols());
		return m_j;
	}

	std::vector<int> HoQpController::GetContactLegs(
			const Eigen::VectorXd &contact_pattern
			)
	{
		std::vector<int> contact_legs;
		for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
		{
			if (contact_pattern[leg_i] == 1)
				contact_legs.push_back(leg_i);
		}
		return contact_legs;
	}

	std::vector<int> HoQpController::GetSwingLegs(
			const Eigen::VectorXd &contact_pattern
			)
	{
		std::vector<int> swing_legs;
		for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
		{
			if (contact_pattern[leg_i] == 0)
				swing_legs.push_back(leg_i);
		}
		return swing_legs;
	}

	Eigen::VectorXd HoQpController::GetSwingLegCmd(
			const Eigen::VectorXd &legs_cmd,
			const std::vector<int> &swing_legs,
			const int num_swing_legs
			)
	{
		Eigen::VectorXd swing_legs_cmd(num_swing_legs * k3D);
		for (int i = 0; i < num_swing_legs_; ++i)
		{
			auto swing_leg_cmd =
				legs_cmd.block<k3D,1>(k3D * swing_legs[i],0);
			swing_legs_cmd.block<k3D,1>(k3D * i,0) = swing_leg_cmd;
		}
		return swing_legs_cmd;
	}
}
