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
		if (!run_once_)
		{
			UpdateDynamicsTerms(q,u);
			TaskDefinition fb_eom_task = ConstructFloatingBaseEomTask();

			Eigen::VectorXd sol_lp = SolveWithLinearProgram(fb_eom_task);
			CheckSolutionValid(fb_eom_task, sol_lp);

			PrintTask(fb_eom_task);
			HoQpProblem fb_eom_prob(fb_eom_task);
			Eigen::VectorXd sol = fb_eom_prob.GetSolution();
			CheckSolutionValid(fb_eom_task, sol);

			run_once_ = true;
		}
	}


	TaskDefinition HoQpController::ConstructFloatingBaseEomTask()
	{
		Eigen::MatrixXd mass_matrix_fb =
			GetFloatingBaseMatrix(mass_matrix_);
		Eigen::MatrixXd bias_vector_fb =
			GetFloatingBaseMatrix(bias_vector_);
		Eigen::MatrixXd contact_jacobian_transpose =
			contact_jacobian_.transpose();
		Eigen::MatrixXd contact_jacobian_fb_t
			= GetFloatingBaseMatrix(contact_jacobian_transpose);

		Eigen::MatrixXd A(kNumTwistCoords,num_decision_vars_);
		A << mass_matrix_fb, -contact_jacobian_fb_t; 

		Eigen::VectorXd b(kNumTwistCoords);
		b << -bias_vector_fb;

		TaskDefinition fb_eom_constraint = {.A=A, .b=b};
		return fb_eom_constraint;
	}

	Eigen::MatrixXd HoQpController::GetFloatingBaseMatrix(
			Eigen::MatrixXd &m
			)
	{
		Eigen::MatrixXd m_fb
			= m.block(0,0,kNumTwistCoords,m.cols());
		return m_fb;
	}


	void HoQpController::UpdateDynamicsTerms(
			Eigen::Matrix<double,kNumGenCoords, 1> q,
			Eigen::Matrix<double,kNumGenVels, 1> u
			)
	{
		// TODO: Assumes 4 contact points!
		mass_matrix_ = robot_dynamics_.GetMassMatrix(q);
		bias_vector_ = robot_dynamics_.GetBiasVector(q,u);
		contact_jacobian_ = robot_dynamics_.GetStackedContactJacobianPos(q);
	}

	// ******* //
	// TESTING //
	// ******* //
	// TODO: Just placeholder test code
	void HoQpController::TestSingleEqTask()
	{
		int num_decision_vars = 5;

		Eigen::MatrixXd A(2,num_decision_vars);
		A << -1,1,0,3,3,
					0,2,3,1,0;
		Eigen::VectorXd b(2);
		b << 1, 10;

		TaskDefinition test_task_eq = {.A=A, .b=b};

		Eigen::VectorXd sol = SolveWithLinearProgram(test_task_eq);
		CheckSolutionValid(test_task_eq, sol);

		HoQpProblem test_qp_problem = HoQpProblem(test_task_eq);

		Eigen::VectorXd sol_ho_qp = test_qp_problem.GetSolution();
		CheckSolutionValid(test_task_eq, sol_ho_qp);
	}
	//
	void HoQpController::TestTwoTasksEqFirst()
	{
		int num_decision_vars = 2;

		Eigen::MatrixXd D1(1,num_decision_vars);
		D1 << 1,1;
		Eigen::VectorXd f1(1);
		f1 << 5;
		TaskDefinition test_task_ineq = {.D=D1, .f=f1};

		Eigen::MatrixXd A2(1,num_decision_vars);
		A2 << -1,1;
		Eigen::VectorXd b2(1);
		b2 << 1;
		TaskDefinition test_task_eq = {.A=A2, .b=b2};

		Eigen::MatrixXd A = A2;
		Eigen::VectorXd b = b2;
		Eigen::MatrixXd D = D1;
		Eigen::VectorXd f = f1;

		Eigen::VectorXd sol = SolveWithLinearProgram(A, b, D, f);

		CheckSolutionValid(A, b, D, f, sol);

		HoQpProblem test_qp_problem_1 = HoQpProblem(test_task_eq);
		HoQpProblem test_qp_problem_2 =
			HoQpProblem(test_task_ineq, &test_qp_problem_1);

		Eigen::VectorXd sol_ho_qp = test_qp_problem_2.GetSolution();
		CheckSolutionValid(A, b, D, f, sol_ho_qp);
	}

	void HoQpController::TestTwoTasksIneqFirst()
	{
		int num_decision_vars = 2;

		Eigen::MatrixXd D1(1,num_decision_vars);
		D1 << 1,1;
		Eigen::VectorXd f1(1);
		f1 << 5;
		TaskDefinition test_task_ineq = {.D=D1, .f=f1};

		Eigen::MatrixXd A2(1,num_decision_vars);
		A2 << -1,1;
		Eigen::VectorXd b2(1);
		b2 << 1;
		TaskDefinition test_task_eq = {.A=A2, .b=b2};

		Eigen::MatrixXd A = A2;
		Eigen::VectorXd b = b2;
		Eigen::MatrixXd D = D1;
		Eigen::VectorXd f = f1;

		Eigen::VectorXd sol = SolveWithLinearProgram(A, b, D, f);

		CheckSolutionValid(A, b, D, f, sol);

		HoQpProblem test_qp_problem_1 = HoQpProblem(test_task_ineq);
		HoQpProblem test_qp_problem_2 =
			HoQpProblem(test_task_eq, &test_qp_problem_1);

		Eigen::VectorXd sol_ho_qp = test_qp_problem_2.GetSolution();
		CheckSolutionValid(A, b, D, f, sol_ho_qp);
	}

	void HoQpController::TestThreeTasks()
	{
		int num_decision_vars = 2;

		Eigen::MatrixXd D1(1,num_decision_vars);
		D1 << 1,1;
		Eigen::VectorXd f1(1);
		f1 << 11;
		TaskDefinition test_task_ineq = {.D=D1, .f=f1};

		Eigen::MatrixXd A2(1,num_decision_vars);
		A2 << -1,1;
		Eigen::VectorXd b2(1);
		b2 << 1;
		TaskDefinition test_task_eq = {.A=A2, .b=b2};

		Eigen::MatrixXd A3(1,num_decision_vars);
		A3 << 1,1;
		Eigen::VectorXd b3(1);
		b3 << 10;
		TaskDefinition test_task_eq_2 = {.A=A3, .b=b3};

		Eigen::MatrixXd A = ConcatenateMatrices(A2, A3);
		Eigen::VectorXd b = ConcatenateVectors(b2, b3);
		Eigen::MatrixXd D = D1;
		Eigen::VectorXd f = f1;

		Eigen::VectorXd sol = SolveWithLinearProgram(A, b, D, f);

		CheckSolutionValid(A, b, D, f, sol);

		HoQpProblem test_qp_problem_1 = HoQpProblem(test_task_ineq);
		HoQpProblem test_qp_problem_2 =
			HoQpProblem(test_task_eq, &test_qp_problem_1);
		HoQpProblem test_qp_problem_3 =
			HoQpProblem(test_task_eq_2, &test_qp_problem_2);

		Eigen::VectorXd sol_ho_qp = test_qp_problem_3.GetSolution();
		CheckSolutionValid(A, b, D, f, sol_ho_qp);
	}

	void HoQpController::TestFourTasks()
	{
		int num_decision_vars = 2;

		Eigen::MatrixXd D1(1,num_decision_vars);
		D1 << 1,1;
		Eigen::VectorXd f1(1);
		f1 << 11;
		TaskDefinition test_task_ineq = {.D=D1, .f=f1};

		Eigen::MatrixXd D2(1,num_decision_vars);
		D2 << -1,-1;
		Eigen::VectorXd f2(1);
		f2 << 7;
		TaskDefinition test_task_ineq_2 = {.D=D2, .f=f2};

		Eigen::MatrixXd A2(1,num_decision_vars);
		A2 << -1,1;
		Eigen::VectorXd b2(1);
		b2 << 1;
		TaskDefinition test_task_eq = {.A=A2, .b=b2};

		Eigen::MatrixXd A3(1,num_decision_vars);
		A3 << 1,1;
		Eigen::VectorXd b3(1);
		b3 << 10;
		TaskDefinition test_task_eq_2 = {.A=A3, .b=b3};

		Eigen::MatrixXd A = ConcatenateMatrices(A2, A3);
		Eigen::VectorXd b = ConcatenateVectors(b2, b3);
		Eigen::MatrixXd D = ConcatenateMatrices(D1, D2);
		Eigen::VectorXd f = ConcatenateVectors(f1, f2);

		Eigen::VectorXd sol = SolveWithLinearProgram(A, b, D, f);

		CheckSolutionValid(A, b, D, f, sol);

//		HoQpProblem test_qp_problem_1 = HoQpProblem(test_task_ineq);
//		HoQpProblem test_qp_problem_2 =
//			HoQpProblem(test_task_eq, &test_qp_problem_1);
//		HoQpProblem test_qp_problem_3 =
//			HoQpProblem(test_task_eq_2, &test_qp_problem_2);
//		HoQpProblem test_qp_problem_4 =
//			HoQpProblem(test_task_ineq_2, &test_qp_problem_3);
//
//		Eigen::VectorXd sol_ho_qp = test_qp_problem_4.GetSolution();
//		CheckSolutionValid(A, b, D, f, sol_ho_qp);

		HoQpProblem second_test_qp_problem_1 = HoQpProblem(test_task_eq);
		HoQpProblem second_test_qp_problem_2 =
			HoQpProblem(test_task_ineq_2, &second_test_qp_problem_1);
		HoQpProblem second_test_qp_problem_3 =
			HoQpProblem(test_task_ineq, &second_test_qp_problem_2);
		HoQpProblem second_test_qp_problem_4 =
			HoQpProblem(test_task_eq_2, &second_test_qp_problem_3);

		Eigen::VectorXd second_sol_ho_qp = second_test_qp_problem_4.GetSolution();
		CheckSolutionValid(A, b, D, f, second_sol_ho_qp);
	}

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
