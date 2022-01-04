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
		// TODO: Testing
		Eigen::MatrixXd A1 = Eigen::MatrixXd::Random(1,num_decision_vars_);
		Eigen::VectorXd b1 = Eigen::VectorXd::Random(1);
		Eigen::MatrixXd D1 = Eigen::MatrixXd::Random(2,num_decision_vars_);
		Eigen::VectorXd f1 = Eigen::VectorXd::Random(2);

		Eigen::MatrixXd A2 = Eigen::MatrixXd::Random(3,num_decision_vars_);
		Eigen::VectorXd b2 = Eigen::VectorXd::Random(3);
		Eigen::MatrixXd D2 = Eigen::MatrixXd::Random(4,num_decision_vars_);
		Eigen::VectorXd f2 = Eigen::VectorXd::Random(4);

		Eigen::MatrixXd A3 = Eigen::MatrixXd::Random(5,num_decision_vars_);
		Eigen::VectorXd b3 = Eigen::VectorXd::Random(5);
		Eigen::MatrixXd D3 = Eigen::MatrixXd::Random(6,num_decision_vars_);
		Eigen::VectorXd f3 = Eigen::VectorXd::Random(6);

		TaskDefinition test_task = {A1, b1, D1, f1};
		TaskDefinition test_task_2 = {A2, b2, D2, f2};
		TaskDefinition test_task_3 = {A3, b3, D3, f3};

		HoQpProblem test_qp_problem = HoQpProblem(test_task);
		HoQpProblem test_qp_problem_2 =
			HoQpProblem(test_task_2, &test_qp_problem);
		HoQpProblem test_qp_problem_3 =
			HoQpProblem(test_task_3, &test_qp_problem_2);

		TaskDefinition test = test_qp_problem_3.GetAccumTasks();
		PrintMatrixSize("A", test.A);
		PrintMatrixSize("b", test.b);
		PrintMatrixSize("D", test.D);
		PrintMatrixSize("f", test.f);
	}
}

