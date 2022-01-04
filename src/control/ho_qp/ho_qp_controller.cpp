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
		Eigen::MatrixXd A1(1,1);
		A1 << 1;
		Eigen::VectorXd b1(1);
		b1 << 0;
		Eigen::MatrixXd D1(2,1);
		D1 << -1,
					 1;
		Eigen::VectorXd f1(2);
		f1 << -2,
					 3;

		TaskDefinition test_task = {A1, b1, D1, f1};

		HoQpProblem test_qp_problem = HoQpProblem(test_task);

//		HoQpProblem test_qp_problem_2 =
//			HoQpProblem(test_task_2, &test_qp_problem);
//		HoQpProblem test_qp_problem_3 =
//			HoQpProblem(test_task_3, &test_qp_problem_2);
	}
}

