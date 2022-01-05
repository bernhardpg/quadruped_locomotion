#include "control/ho_qp/ho_qp_controller.hpp"

namespace control
{
	HoQpController::HoQpController()
		: HoQpController(3) // TODO: Default number of tasks
	{}

	HoQpController::HoQpController(int num_tasks)
		: num_tasks_(num_tasks)
	{
		//num_contacts_ = 4; // TODO: generalize this
		//num_decision_vars_ = kNumGenVels + kNumPosDims * num_contacts_;

//		TestTwoTasksEqFirst();
//		TestTwoTasksEqFirst();
		//TestThreeTasks();
		TestFourTasks();
	}

	// ******* //
	// TESTING //
	// ******* //
	// TODO: Just placeholder test code
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

	// TODO remove this?
	Eigen::VectorXd HoQpController::SolveWithLinearProgram(TaskDefinition task)
	{
		std::cout << "---- Solving as LP ----" << std::endl;
		drake::solvers::MathematicalProgram prog;
		auto x = prog.NewContinuousVariables(task.A.cols(), 1, "x");
		if (task.A.rows() > 0)
			prog.AddLinearConstraint(task.A * x == task.b);
		if (task.D.rows() > 0)
		prog.AddLinearConstraint(task.D * x <= task.f);

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

}
