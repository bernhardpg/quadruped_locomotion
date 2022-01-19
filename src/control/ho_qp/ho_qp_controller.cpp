#include "control/ho_qp/ho_qp_controller.hpp"

namespace control
{
	HoQpController::HoQpController() {}

	void HoQpController::CalcJointCmd(
			Eigen::Matrix<double,kNumGenCoords, 1> q,
			Eigen::Matrix<double,kNumGenVels, 1> u
			)
	{
		if (run_once_ == false)
		{
			std::cout << "Starting update\n";
			UpdateModelDynamics(q,u);
			std::cout << "Creating tasks\n";
			std::vector<TaskDefinition> tasks = ConstructTasks(q,u);
			std::cout << "Created tasks\n";
			std::vector<std::shared_ptr<HoQpProblem>>
				opt_problems = ConstructOptProblems(tasks);

			solution_ = opt_problems.back()->GetSolution();
			q_j_ddot_cmd_ = solution_.block<kNumJoints,1>(kNumTwistCoords,0);
			CalcJointTorquesCmd();
		}
	}

	void HoQpController::SetBaseCmd(
			Eigen::VectorXd r_cmd,
			Eigen::VectorXd r_dot_cmd,
			Eigen::VectorXd r_ddot_cmd
			)
	{
		r_cmd_ = r_cmd; 
		r_dot_cmd_ = r_dot_cmd;
		r_ddot_cmd_ = r_ddot_cmd;
	}

	void HoQpController::SetLegCmd(
			Eigen::VectorXd r_c_cmd,
			Eigen::VectorXd r_c_dot_cmd,
			Eigen::VectorXd r_c_ddot_cmd,
			std::vector<int> legs_in_contact
			)
	{
		r_c_cmd_ = r_c_cmd;
		r_c_dot_cmd_ = r_c_dot_cmd;
		r_c_ddot_cmd_ = r_c_ddot_cmd;
		legs_in_contact_ = legs_in_contact;
		num_contacts_ = legs_in_contact_.size();
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
		std::cout << "q:\n";
		PrintMatrix(q);
		robot_dynamics_.SetState(q,u);

		M_ = robot_dynamics_.GetMassMatrix();
		M_j_ = GetJointRows(M_);
		std::cout << "1\n";

		c_ = robot_dynamics_.GetBiasVector();
		c_j_ = GetJointRows(c_);
		std::cout << "2\n";

		J_c_ = robot_dynamics_.GetStackedContactJacobianInW(legs_in_contact_);
		std::cout << "3\n";
		J_c_dot_u_ = robot_dynamics_.GetStackedContactAccInW(legs_in_contact_);
		std::cout << "4\n";
		Eigen::MatrixXd J_c_t = J_c_.transpose();
		J_c_j_t_ = GetJointRows(J_c_t);
		std::cout << "5\n";

		auto J_b = robot_dynamics_.GetBaseJacobianInW();
		std::cout << "J_b" << std::endl;
		PrintMatrix(J_b);
		std::cout << "6\n";
		J_b_rot_ = J_b.block(0,0,3,kNumGenVels);
		J_b_pos_ = J_b.block(3,0,3,kNumGenVels);
		std::cout << "7\n";
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
		std::cout << "fb_eom_task\n";
		PrintTask(fb_eom_task);

		TaskDefinition joint_torque_task = ConstructJointTorqueTask();
		std::cout << "joint_torque_task\n";
		PrintTask(joint_torque_task);
		TaskDefinition friction_cone_task = ConstructFrictionConeTask();
		std::cout << "friction_cone_task\n";
		PrintTask(friction_cone_task);
		TaskDefinition joint_torque_and_friction_task =
			ConcatenateTasks(joint_torque_task, friction_cone_task);

		TaskDefinition no_contact_motion_task =
			ConstructNoContactMotionTask();
		std::cout << "no_contact_motion_task\n";
		PrintTask(no_contact_motion_task);

		TaskDefinition base_pos_traj_task =
			ConstructBasePosTrajTask(q,u);
		std::cout << "base_pos_traj_task\n";
		PrintTask(base_pos_traj_task);
		TaskDefinition base_rot_traj_task =
			ConstructBaseRotTrajTask(q,u);
		std::cout << "base_rot_traj_task\n";
		PrintTask(base_rot_traj_task);
		TaskDefinition base_traj_task = 
			ConcatenateTasks(base_pos_traj_task, base_rot_traj_task);

		TaskDefinition force_min_task = ConstructForceMinimizationTask();
		std::cout << "force_min_task";
		PrintTask(force_min_task);

		std::vector<TaskDefinition> tasks{
			fb_eom_task,
			joint_torque_and_friction_task,
			no_contact_motion_task,
			base_traj_task,
			force_min_task,
		};

		return tasks;
	}

	TaskDefinition HoQpController::ConstructBasePosTrajTask(
			const Eigen::VectorXd &q, const Eigen::VectorXd &u 
			)
	{
		const Eigen::VectorXd r_IB_I = q.block(kQuatSize,0,k3D,1);
		const Eigen::VectorXd v_IB_I = u.block(k3D,0,k3D,1);

		double k_pos = 1.0;
		double k_vel = 1.0;

		Eigen::MatrixXd zero = 
			Eigen::MatrixXd::Zero(
					J_b_pos_.rows(), k3D * num_contacts_
					);

		Eigen::MatrixXd A(J_b_pos_.rows(), num_decision_vars_);
		A << J_b_pos_, zero;

		Eigen::VectorXd b(J_b_pos_.rows());
		b << r_ddot_cmd_
			+ k_vel * (r_dot_cmd_ - v_IB_I);
			//+ k_pos * (r_cmd_ - r_IB_I);

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

		const double k_pos = 1.0; // TODO: tune these
		const double k_vel = 0.1;

		Eigen::MatrixXd zero = 
			Eigen::MatrixXd::Zero(
					J_b_rot_.rows(), k3D * num_contacts_
					);

		Eigen::MatrixXd A(J_b_rot_.rows(), num_decision_vars_);
		A << J_b_rot_, zero;

		Eigen::VectorXd b(J_b_pos_.rows());
		b << k_vel * (wd_IB - w_IB);  // TODO: implement quaternion error here

		TaskDefinition base_rot_traj_task = {.A=A, .b=b};
		return base_rot_traj_task;
	}

	TaskDefinition HoQpController::ConstructNoContactMotionTask()
	{
		Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(
				k3D * num_contacts_, k3D * num_contacts_
				);

		Eigen::MatrixXd A(k3D * num_contacts_, num_decision_vars_);
		PrintMatrixSize("J_c_", J_c_);
		PrintMatrixSize("zero", zero);
		A << J_c_, zero;
		std::cout << "A\n";
		PrintMatrix(A);

		Eigen::VectorXd b(k3D * num_contacts_);
		b << -J_c_dot_u_;
		std::cout << "b\n";
		PrintMatrix(b);

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
				 kNumTwistCoords + legs_in_contact_[i] * k3D)
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
}
