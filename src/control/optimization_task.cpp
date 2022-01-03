#include "control/optimization_task.hpp"

namespace control
{
	HoQpProblem::HoQpProblem()
	{
		ROS_INFO("Created new empty HoQpProblem");
	}

	void HoQpProblem::SetTask(TaskDefinition new_task)
	{
		curr_task_ = new_task;
	}

	void HoQpProblem::SetHigherPriorityTask(
			TaskDefinition higher_pri_task
			)
	{
		higher_pri_task_ = higher_pri_task;
	}

	TaskDefinition HoQpProblem::GetAccumulatedTask()
	{
		if (!IsHigherPriTaskDefined()) return curr_task_;
		TaskDefinition accumulated_task =
			ConcatenateTasks(curr_task_, higher_pri_task_);
		return accumulated_task;
	}

	TaskDefinition HoQpProblem::ConcatenateTasks(
			TaskDefinition &t1, TaskDefinition &t2
			)
	{
		Eigen::MatrixXd A_concat = 
			ConcatenateMatrices(t1.A, t2.A);
		Eigen::VectorXd b_concat =
			ConcatenateVectors(t1.b, t2.b);
		Eigen::MatrixXd D_concat =
			ConcatenateMatrices(t1.D, t2.D);
		Eigen::VectorXd f_concat =
			ConcatenateVectors(t1.f, t2.f);

		TaskDefinition res =
			{A_concat, b_concat, D_concat, f_concat};
		
		return res;
	}

	Eigen::MatrixXd HoQpProblem::ConcatenateMatrices(
			Eigen::MatrixXd &m1, Eigen::MatrixXd &m2
			)
	{
		assert (m1.cols() == m2.cols());
		Eigen::MatrixXd res(m1.rows() + m2.rows(), m1.cols());
		res << m1,
					 m2;

		return res;
	}

	Eigen::VectorXd HoQpProblem::ConcatenateVectors(
			Eigen::VectorXd &v1, Eigen::VectorXd &v2
			)
	{
		assert (v1.cols() == v2.cols());
		Eigen::VectorXd res(v1.rows() + v2.rows());
		res << v1,
					 v2;

		return res;
	}

	bool HoQpProblem::IsHigherPriTaskDefined()
	{
		bool higher_pri_task_defined = higher_pri_task_.A.rows() > 0;
		return higher_pri_task_defined;
	}
}

