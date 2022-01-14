#include "control/ho_qp/ho_qp_controller.hpp"

namespace control
{
	HoQpController::HoQpController()
		: HoQpController(3) // TODO: Default number of tasks
	{}

	HoQpController::HoQpController(int num_tasks)
		: num_tasks_(num_tasks)
	{
		num_contacts_ = 4; // TODO: generalize this
		num_decision_vars_ = kNumGenVels + kNumPosDims * num_contacts_;
		ROS_INFO("num_decision_vars: %d", num_decision_vars_);
	}

	void HoQpController::Update(
			Eigen::Matrix<double,kNumGenCoords, 1> q,
			Eigen::Matrix<double,kNumGenVels, 1> u
			)
	{
		bool run_once = false;
		if (!run_once)
		{
			ROS_INFO("===== NEW UPDATE =====");
			UpdateDynamicsTerms(q,u);
			std::vector<TaskDefinition> tasks = ConstructTasks(q,u);
			std::vector<std::shared_ptr<HoQpProblem>>
				opt_problems = ConstructOptProblems(tasks);
			Eigen::VectorXd sol = opt_problems.back()->GetSolution();

			CheckSolutionValid(tasks[0], sol);
			//CheckSolutionValid(tasks[1], sol);
			//CheckSolutionValid(tasks[2], sol);

			q_j_ddot_cmd_ = sol.block(kNumTwistCoords,0,kNumJoints,1);
			std::cout << "q_j_ddot:\n";
			PrintMatrix(q_j_ddot_cmd_.transpose());
		}
	}

	Eigen::VectorXd HoQpController::GetJointAccelerationCmd()
	{
		return q_j_ddot_cmd_;
	}

	// ********************* //
	// DYNAMICS & KINEMATICS // 
	// ********************* //

	void HoQpController::UpdateDynamicsTerms(
			Eigen::Matrix<double,kNumGenCoords, 1> q,
			Eigen::Matrix<double,kNumGenVels, 1> u
			)
	{
		// TODO: Assumes 4 contact points!
		M_ = robot_dynamics_.GetMassMatrix(q);
		c_ = robot_dynamics_.GetBiasVector(q,u);
		J_c_ = robot_dynamics_.GetStackedContactJacobianPos(q);
		J_c_dot_u_ = robot_dynamics_.GetContactAccPosStacked(q,u);

		auto J_b = robot_dynamics_.GetBaseJacobian(q);
		J_b_pos_ = J_b.block(0,0,2,kNumGenVels); // only keep x, y
		J_b_rot_ = J_b.block(kNumPosDims,0,2,kNumGenVels); // only keep roll, pitch
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

		auto fb_lin_vel = u.block(0,0,2,1);
		TaskDefinition com_pos_traj_task =
			ConstructComPosTrajTask(fb_lin_vel);
		auto fb_ang_vel = u.block(kNumPosDims,0,2,1);
		TaskDefinition com_rot_traj_task =
			ConstructComRotTrajTask(fb_ang_vel);
		TaskDefinition com_traj_task = 
			ConcatenateTasks(com_pos_traj_task, com_rot_traj_task);

		TaskDefinition force_min_task = ConstructForceMinimizationTask();
		TaskDefinition acc_min_task = ConstructJointAccMinimizationTask();

		std::vector<TaskDefinition> tasks{
			//fb_eom_task,
			//joint_torque_and_friction_task,
			no_contact_motion_task,
			//force_min_task,
			//acc_min_task,
			//com_traj_task,
		};

		return tasks;
	}

	TaskDefinition HoQpController::ConstructComPosTrajTask(
			Eigen::VectorXd com_vel 
			)
	{
		Eigen::VectorXd com_vel_des(2);
		com_vel_des << 1, 0; // TODO: Take from traj generator

		double k_pos = 1.0;
		double k_vel = 0.1;

		Eigen::MatrixXd zero = 
			Eigen::MatrixXd::Zero(
					J_b_pos_.rows(), kNumPosDims * num_contacts_
					);

		Eigen::MatrixXd A(J_b_pos_.rows(), num_decision_vars_);
		A << J_b_pos_, zero;

		Eigen::VectorXd b(J_b_pos_.rows());
		b << k_vel * (com_vel_des - com_vel); // TODO: missing a bunch of terms

		TaskDefinition com_pos_traj_task = {.A=A, .b=b};
		return com_pos_traj_task;
	}

	TaskDefinition HoQpController::ConstructComRotTrajTask(
			Eigen::VectorXd ang_vel
			)
	{
		Eigen::VectorXd com_vel_des(2);
		com_vel_des << 0, 0;

		double k_pos = 1.0;
		double k_vel = 0.1;

		Eigen::MatrixXd zero = 
			Eigen::MatrixXd::Zero(
					J_b_rot_.rows(), kNumPosDims * num_contacts_
					);

		Eigen::MatrixXd A(J_b_rot_.rows(), num_decision_vars_);
		A << J_b_rot_, zero;

		Eigen::VectorXd b(J_b_pos_.rows());
		b << -k_vel * (com_vel_des); // TODO: missing a bunch of terms here

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

		TaskDefinition fb_eom_constraint = {.A=A, .b=b};
		return fb_eom_constraint;
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


	// ******* //
	// TESTING //
	// ******* //
	// TODO: Just placeholder test code
//	void HoQpController::TestEomConstraint()
//	{
//		TaskDefinition fb_eom_task = ConstructFloatingBaseEomTask();
//
//		Eigen::VectorXd sol_lp = SolveWithLinearProgram(fb_eom_task);
//		CheckSolutionValid(fb_eom_task, sol_lp);
//
//		PrintTask(fb_eom_task);
//		HoQpProblem fb_eom_prob(fb_eom_task);
//		Eigen::VectorXd sol = fb_eom_prob.GetSolution();
//		CheckSolutionValid(fb_eom_task, sol);
//
//		run_once_ = true;
//	}
//	void HoQpController::TestSingleEqTask()
//	{
//		int num_decision_vars = 5;
//
//		Eigen::MatrixXd A(2,num_decision_vars);
//		A << -1,1,0,3,3,
//					0,2,3,1,0;
//		Eigen::VectorXd b(2);
//		b << 1, 10;
//
//		TaskDefinition test_task_eq = {.A=A, .b=b};
//
//		Eigen::VectorXd sol = SolveWithLinearProgram(test_task_eq);
//		CheckSolutionValid(test_task_eq, sol);
//
//		HoQpProblem test_qp_problem = HoQpProblem(test_task_eq);
//
//		Eigen::VectorXd sol_ho_qp = test_qp_problem.GetSolution();
//		CheckSolutionValid(test_task_eq, sol_ho_qp);
//	}
//	//
//	void HoQpController::TestTwoTasksEqFirst()
//	{
//		int num_decision_vars = 2;
//
//		Eigen::MatrixXd D1(1,num_decision_vars);
//		D1 << 1,1;
//		Eigen::VectorXd f1(1);
//		f1 << 5;
//		TaskDefinition test_task_ineq = {.D=D1, .f=f1};
//
//		Eigen::MatrixXd A2(1,num_decision_vars);
//		A2 << -1,1;
//		Eigen::VectorXd b2(1);
//		b2 << 1;
//		TaskDefinition test_task_eq = {.A=A2, .b=b2};
//
//		Eigen::MatrixXd A = A2;
//		Eigen::VectorXd b = b2;
//		Eigen::MatrixXd D = D1;
//		Eigen::VectorXd f = f1;
//
//		Eigen::VectorXd sol = SolveWithLinearProgram(A, b, D, f);
//
//		CheckSolutionValid(A, b, D, f, sol);
//
//		HoQpProblem test_qp_problem_1 = HoQpProblem(test_task_eq);
//		HoQpProblem test_qp_problem_2 =
//			HoQpProblem(test_task_ineq, &test_qp_problem_1);
//
//		Eigen::VectorXd sol_ho_qp = test_qp_problem_2.GetSolution();
//		CheckSolutionValid(A, b, D, f, sol_ho_qp);
//	}
//
//	void HoQpController::TestTwoTasksIneqFirst()
//	{
//		int num_decision_vars = 2;
//
//		Eigen::MatrixXd D1(1,num_decision_vars);
//		D1 << 1,1;
//		Eigen::VectorXd f1(1);
//		f1 << 5;
//		TaskDefinition test_task_ineq = {.D=D1, .f=f1};
//
//		Eigen::MatrixXd A2(1,num_decision_vars);
//		A2 << -1,1;
//		Eigen::VectorXd b2(1);
//		b2 << 1;
//		TaskDefinition test_task_eq = {.A=A2, .b=b2};
//
//		Eigen::MatrixXd A = A2;
//		Eigen::VectorXd b = b2;
//		Eigen::MatrixXd D = D1;
//		Eigen::VectorXd f = f1;
//
//		Eigen::VectorXd sol = SolveWithLinearProgram(A, b, D, f);
//
//		CheckSolutionValid(A, b, D, f, sol);
//
//		HoQpProblem test_qp_problem_1 = HoQpProblem(test_task_ineq);
//		HoQpProblem test_qp_problem_2 =
//			HoQpProblem(test_task_eq, &test_qp_problem_1);
//
//		Eigen::VectorXd sol_ho_qp = test_qp_problem_2.GetSolution();
//		CheckSolutionValid(A, b, D, f, sol_ho_qp);
//	}
//
//	void HoQpController::TestThreeTasks()
//	{
//		int num_decision_vars = 2;
//
//		Eigen::MatrixXd D1(1,num_decision_vars);
//		D1 << 1,1;
//		Eigen::VectorXd f1(1);
//		f1 << 11;
//		TaskDefinition test_task_ineq = {.D=D1, .f=f1};
//
//		Eigen::MatrixXd A2(1,num_decision_vars);
//		A2 << -1,1;
//		Eigen::VectorXd b2(1);
//		b2 << 1;
//		TaskDefinition test_task_eq = {.A=A2, .b=b2};
//
//		Eigen::MatrixXd A3(1,num_decision_vars);
//		A3 << 1,1;
//		Eigen::VectorXd b3(1);
//		b3 << 10;
//		TaskDefinition test_task_eq_2 = {.A=A3, .b=b3};
//
//		Eigen::MatrixXd A = ConcatenateMatrices(A2, A3);
//		Eigen::VectorXd b = ConcatenateVectors(b2, b3);
//		Eigen::MatrixXd D = D1;
//		Eigen::VectorXd f = f1;
//
//		Eigen::VectorXd sol = SolveWithLinearProgram(A, b, D, f);
//
//		CheckSolutionValid(A, b, D, f, sol);
//
//		HoQpProblem test_qp_problem_1 = HoQpProblem(test_task_ineq);
//		HoQpProblem test_qp_problem_2 =
//			HoQpProblem(test_task_eq, &test_qp_problem_1);
//		HoQpProblem test_qp_problem_3 =
//			HoQpProblem(test_task_eq_2, &test_qp_problem_2);
//
//		Eigen::VectorXd sol_ho_qp = test_qp_problem_3.GetSolution();
//		CheckSolutionValid(A, b, D, f, sol_ho_qp);
//	}
//
//	void HoQpController::TestFourTasks()
//	{
//		int num_decision_vars = 2;
//
//		Eigen::MatrixXd D1(1,num_decision_vars);
//		D1 << 1,1;
//		Eigen::VectorXd f1(1);
//		f1 << 11;
//		TaskDefinition test_task_ineq = {.D=D1, .f=f1};
//
//		Eigen::MatrixXd D2(1,num_decision_vars);
//		D2 << -1,-1;
//		Eigen::VectorXd f2(1);
//		f2 << 7;
//		TaskDefinition test_task_ineq_2 = {.D=D2, .f=f2};
//
//		Eigen::MatrixXd A2(1,num_decision_vars);
//		A2 << -1,1;
//		Eigen::VectorXd b2(1);
//		b2 << 1;
//		TaskDefinition test_task_eq = {.A=A2, .b=b2};
//
//		Eigen::MatrixXd A3(1,num_decision_vars);
//		A3 << 1,1;
//		Eigen::VectorXd b3(1);
//		b3 << 10;
//		TaskDefinition test_task_eq_2 = {.A=A3, .b=b3};
//
//		Eigen::MatrixXd A = ConcatenateMatrices(A2, A3);
//		Eigen::VectorXd b = ConcatenateVectors(b2, b3);
//		Eigen::MatrixXd D = ConcatenateMatrices(D1, D2);
//		Eigen::VectorXd f = ConcatenateVectors(f1, f2);
//
//		Eigen::VectorXd sol = SolveWithLinearProgram(A, b, D, f);
//
//		CheckSolutionValid(A, b, D, f, sol);
//
////		HoQpProblem test_qp_problem_1 = HoQpProblem(test_task_ineq);
////		HoQpProblem test_qp_problem_2 =
////			HoQpProblem(test_task_eq, &test_qp_problem_1);
////		HoQpProblem test_qp_problem_3 =
////			HoQpProblem(test_task_eq_2, &test_qp_problem_2);
////		HoQpProblem test_qp_problem_4 =
////			HoQpProblem(test_task_ineq_2, &test_qp_problem_3);
////
////		Eigen::VectorXd sol_ho_qp = test_qp_problem_4.GetSolution();
////		CheckSolutionValid(A, b, D, f, sol_ho_qp);
//
//		HoQpProblem second_test_qp_problem_1 = HoQpProblem(test_task_eq);
//		HoQpProblem second_test_qp_problem_2 =
//			HoQpProblem(test_task_ineq_2, &second_test_qp_problem_1);
//		HoQpProblem second_test_qp_problem_3 =
//			HoQpProblem(test_task_ineq, &second_test_qp_problem_2);
//		HoQpProblem second_test_qp_problem_4 =
//			HoQpProblem(test_task_eq_2, &second_test_qp_problem_3);
//
//		Eigen::VectorXd second_sol_ho_qp = second_test_qp_problem_4.GetSolution();
//		CheckSolutionValid(A, b, D, f, second_sol_ho_qp);
//	}

	Eigen::VectorXd HoQpController::SolveWithLinearProgram(
			Eigen::MatrixXd A, Eigen::VectorXd b,
			Eigen::MatrixXd D, Eigen::VectorXd f
			)
	{
		std::cout << "---- Solving as LP ----" << std::endl;
		drake::solvers::MathematicalProgram prog;
		auto x = prog.NewContinuousVariables(A.cols(), 1, "x");
		if (A.rows() > 0)
			prog.AddLinearConstraint(A * x == b);
		if (D.rows() > 0)
		prog.AddLinearConstraint(D * x <= f);

		std::cout << "Solution to LP" << std::endl;
		std::cout << prog.to_string() << std::endl;
		auto result = Solve(prog);
		ROS_INFO_STREAM("Solver id: " << result.get_solver_id()
			<< "\nFound solution: " << result.is_success()
			<< "\nSolution result: " << result.get_solution_result()
			<< std::endl);
		assert(result.is_success());
		std::cout << "Result: " << std::endl;
		PrintMatrix(result.GetSolution());

		return result.GetSolution();
	}

	Eigen::VectorXd HoQpController::SolveWithLinearProgram(TaskDefinition task)
	{
		return SolveWithLinearProgram(task.A, task.b, task.D, task.f);
	}

}
