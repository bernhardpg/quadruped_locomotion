#include "control/optimization_task.hpp"

namespace control
{
	HoQpProblem::HoQpProblem()
		: HoQpProblem(TaskDefinition(), nullptr)
	{}

	HoQpProblem::HoQpProblem(TaskDefinition new_task)
		: HoQpProblem(new_task, nullptr)
	{
	}

	HoQpProblem::HoQpProblem(
			TaskDefinition new_task, HoQpProblem *higher_pri_problem
			)
		: curr_task_(new_task), higher_pri_problem_(higher_pri_problem)
	{
		AccumulateTasks();
		CalcNullspaceMatrix();
		PrintMatrixSize("Z", Z_);
	}

	// ***************** //
	// SETTERS & GETTERS //
	// ***************** //

	TaskDefinition HoQpProblem::GetAccumTask()
	{
		return accumulated_task_;
	}

	Eigen::MatrixXd HoQpProblem::GetAccumA()
	{
		return accumulated_task_.A;
	}

	Eigen::MatrixXd HoQpProblem::GetAccumD()
	{
		return accumulated_task_.D;
	}

	Eigen::VectorXd HoQpProblem::GetAccumB()
	{
		return accumulated_task_.b;
	}

	Eigen::VectorXd HoQpProblem::GetAccumF()
	{
		return accumulated_task_.f;
	}

	Eigen::MatrixXd HoQpProblem::GetAccumNullspaceMatrix()
	{
		return Z_;
	}

	// ************** //
	// INITIALIZATION //
	// ************** //

	void HoQpProblem::AccumulateTasks()
	{
		if (!IsHigherPriTaskDefined())
		{
			accumulated_task_ = curr_task_;
		}
		else
		{
			accumulated_task_ = ConcatenateTasks(
					curr_task_, higher_pri_problem_->GetAccumTask()
					);
		}
	}

	void HoQpProblem::CalcNullspaceMatrix()
	{
		if (higher_pri_problem_ == nullptr)
		{
			Z_ = CalcNullSpaceProjMatrix(curr_task_.A);
		}
		else
		{
			auto Z_prev = higher_pri_problem_->GetAccumNullspaceMatrix();
			auto A_curr = accumulated_task_.A;
			Eigen::MatrixXd Null_of_A_curr_times_Z_prev =
				CalcNullSpaceProjMatrix(A_curr * Z_prev);

			Z_ = Z_prev * Null_of_A_curr_times_Z_prev;
		}
	}

	// **************** //
	// HELPER FUNCTIONS //
	// **************** //

	bool HoQpProblem::IsHigherPriTaskDefined()
	{
		return higher_pri_problem_ != nullptr;
	}

	TaskDefinition HoQpProblem::ConcatenateTasks(
			TaskDefinition t1, TaskDefinition t2
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
}

