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

		TestTasks();
	}

	// ******* //
	// TESTING //
	// ******* //

	// TODO: Just placeholder test code
	void HoQpController::TestTasks()
	{
		Eigen::MatrixXd A1(1,2);
		A1 << -1, 1;
		Eigen::VectorXd b1(1);
		b1 << 1;
		Eigen::MatrixXd D1(2,2);
		D1 << 1, 0,
				 -1, 0;
		Eigen::VectorXd f1(2);
		f1 << 10,
				  0;
		TaskDefinition test_task_1 = {A1, b1, D1, f1};
		HoQpProblem test_qp_problem = HoQpProblem(test_task_1);

		Eigen::MatrixXd A2(1,2);
		A2 << 1, 1;
		Eigen::VectorXd b2(1);
		b2 << 5;
		Eigen::MatrixXd D2(1,2);
		D2 << 1, 0;
		Eigen::VectorXd f2(1);
		f2 << 9999;
		TaskDefinition test_task_2 = {A2, b2, D2, f2};

		HoQpProblem test_qp_problem_2 =
			HoQpProblem(test_task_2, &test_qp_problem);

		TestLinearProgram(
				ConcatenateMatrices(A1,A2),
				ConcatenateVectors(b1,b2),
				ConcatenateMatrices(D1,D2),
				ConcatenateVectors(f1,f2)
				);

//		HoQpProblem test_qp_problem_3 =
//			HoQpProblem(test_task_3, &test_qp_problem_2);
	}

	void HoQpController::TestLinearProgram(
			Eigen::MatrixXd A, Eigen::VectorXd b,
			Eigen::MatrixXd D, Eigen::VectorXd f)
	{
		drake::solvers::MathematicalProgram prog;
		auto x = prog.NewContinuousVariables(A.cols(), 1, "x");
		prog.AddLinearConstraint(A * x == b);
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
	}

}
