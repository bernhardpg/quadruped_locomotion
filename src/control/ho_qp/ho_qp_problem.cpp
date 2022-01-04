#include "control/ho_qp/ho_qp_problem.hpp"

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
		num_decision_vars_ = new_task.A.cols();
		num_slack_vars_ = new_task.D.rows();
		CreateDecisionVars();
		CreateSlackVars();

		ROS_INFO("num_decision_vars_: %d", num_decision_vars_);
		ROS_INFO("num_slack_vars_: %d", num_slack_vars_);

		AccumulateTasks();
		ConstructNullspaceMatrix();
		AccumulateSlackSolutions(); // TODO: This should be done after progs are solved
		PrintMatrixSize("Z", Z_);
		ConstructDMatrix();
		PrintMatrixSize("D", D_);
		ConstructFVector();
		PrintMatrixSize("f", f_);

		AddIneqConstraints();
	}

	// ***************** //
	// SETTERS & GETTERS //
	// ***************** //

	TaskDefinition HoQpProblem::GetAccumTasks()
	{
		return accumulated_tasks_;
	}

	Eigen::MatrixXd HoQpProblem::GetAccumA()
	{
		return accumulated_tasks_.A;
	}

	Eigen::MatrixXd HoQpProblem::GetAccumD()
	{
		return accumulated_tasks_.D;
	}

	Eigen::VectorXd HoQpProblem::GetAccumB()
	{
		return accumulated_tasks_.b;
	}

	Eigen::VectorXd HoQpProblem::GetAccumF()
	{
		return accumulated_tasks_.f;
	}

	Eigen::MatrixXd HoQpProblem::GetAccumNullspaceMatrix()
	{
		return Z_;
	}

	int HoQpProblem::GetPrevAccumNumSlackVars()
	{
		if (IsHigherPriProblemDefined())
		{
			return higher_pri_problem_->GetAccumNumSlackVars();
		}
		else
		{
			return 0;
		}
	}

	int HoQpProblem::GetAccumNumSlackVars()
	{
		return accumulated_tasks_.D.rows();	
	}

	// TODO: placeholder
	Eigen::VectorXd HoQpProblem::GetSolution()
	{
		return Eigen::VectorXd::Zero(num_decision_vars_);
	}

	// TODO: placeholder
	Eigen::VectorXd HoQpProblem::GetSlackSolutions()
	{
		return Eigen::VectorXd::Zero(num_slack_vars_);
	}

	Eigen::VectorXd HoQpProblem::GetAccumSlackSolutions()
	{
		return accumulated_slack_vars_;
	}

	symbolic_vector_t HoQpProblem::GetAllDecisionVars()
	{
		int tot_num_decision_vars =
			decision_vars_.rows() + slack_vars_.rows();
		symbolic_vector_t decision_vars(tot_num_decision_vars);
		decision_vars << decision_vars_, 
										 slack_vars_;
		return decision_vars;
	}

	// ********************* //
	// MATRIX INITIALIZATION //
	// ********************* //

	void HoQpProblem::AccumulateTasks()
	{
		if (!IsHigherPriProblemDefined())
		{
			accumulated_tasks_ = curr_task_;
		}
		else
		{
			accumulated_tasks_ = ConcatenateTasks(
					curr_task_, higher_pri_problem_->GetAccumTasks()
					);
		}
	}

	void HoQpProblem::AccumulateSlackSolutions()
	{
		if (!IsHigherPriProblemDefined())		
		{
			accumulated_slack_vars_	= GetSlackSolutions();
		}
		else
		{
			accumulated_slack_vars_	= ConcatenateVectors(
					higher_pri_problem_->GetAccumSlackSolutions(),
					GetSlackSolutions()
					);
		}
	}

	void HoQpProblem::ConstructNullspaceMatrix()
	{
		if (!IsHigherPriProblemDefined())
		{
			Z_ = CalcNullSpaceProjMatrix(curr_task_.A);
		}
		else
		{
			auto Z_prev = higher_pri_problem_->GetAccumNullspaceMatrix();
			auto A_curr = accumulated_tasks_.A;
			Eigen::MatrixXd Null_of_A_curr_times_Z_prev =
				CalcNullSpaceProjMatrix(A_curr * Z_prev);

			Z_ = Z_prev * Null_of_A_curr_times_Z_prev;
		}
	}

	void HoQpProblem::ConstructDMatrix()
	{
		int num_prev_slack_vars = GetPrevAccumNumSlackVars();
		std::cout << "Total previous number of slack variables: " << num_prev_slack_vars << std::endl;

		Eigen::MatrixXd D(
				2 * num_slack_vars_ + num_prev_slack_vars,
				num_decision_vars_ + num_slack_vars_
				);
		Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(
					num_slack_vars_, num_slack_vars_
					);
		Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(
					num_slack_vars_, num_decision_vars_
					);
		Eigen::MatrixXd zero_for_accum_tasks = Eigen::MatrixXd::Zero(
					num_prev_slack_vars, num_slack_vars_
					);

		if (!IsHigherPriProblemDefined())
		{
			D << zero, -eye,
					 curr_task_.D, -eye;
		}
		else
		{
//			PrintMatrixSize("new D",D);
//			PrintMatrixSize("zero",zero);
//			PrintMatrixSize("eye",eye);
//			PrintMatrixSize("curr_task D",curr_task_.D);
//			PrintMatrixSize("accum D",higher_pri_problem_->GetAccumD());
//			PrintMatrixSize("zero acuum",zero_for_accum_tasks);

			// NOTE: This is upside down compared to the paper,
			// but more consistent with the rest of the algorithm
			D << zero, -eye,
					 higher_pri_problem_->GetAccumD(), zero_for_accum_tasks,
					 curr_task_.D, - eye;
		}
		D_ = D;
	}

	void HoQpProblem::ConstructFVector()
	{
		int num_prev_slack_vars = GetPrevAccumNumSlackVars();

		Eigen::VectorXd f(
				2 * num_slack_vars_ + num_prev_slack_vars 
				);
		f.setZero();

		Eigen::VectorXd zero_vec =
			Eigen::VectorXd::Zero(num_slack_vars_);

		if (!IsHigherPriProblemDefined())
		{
			f << zero_vec,
					 curr_task_.f;
		}
		else
		{
			Eigen::VectorXd prev_fs = higher_pri_problem_->GetAccumF();
			Eigen::VectorXd prev_Ds_times_x = higher_pri_problem_->GetAccumD()
				* higher_pri_problem_->GetSolution();
			Eigen::VectorXd prev_accum_slack_vars = 
				higher_pri_problem_->GetAccumSlackSolutions();
			Eigen::VectorXd D_curr_times_x = curr_task_.D
				* higher_pri_problem_->GetSolution();

//			PrintMatrixSize("prev f", prev_fs);
//			PrintMatrixSize("prev_Ds_times_x", prev_Ds_times_x);
//			PrintMatrixSize("prev_accum_slack_variables", prev_accum_slack_vars);
//			PrintMatrixSize("D_curr_times_x", D_curr_times_x);

			f << zero_vec,
				   prev_fs - prev_Ds_times_x + prev_accum_slack_vars,
					 curr_task_.f - D_curr_times_x;
		}
		f_ = f;
	}

	// ******************** //
	// OPTIMIZATION PROBLEM //
	// ******************** //

	void HoQpProblem::CreateDecisionVars()
	{
		decision_vars_ = prog_.NewContinuousVariables(
						num_decision_vars_, 1, "x"
						);
	}

	void HoQpProblem::CreateSlackVars()
	{
		slack_vars_ = prog_.NewContinuousVariables(
						num_slack_vars_, 1, "v"
						);
	}

	void HoQpProblem::AddIneqConstraints()
	{
		symbolic_vector_t decision_vars = GetAllDecisionVars();
		symbolic_vector_t constraint = D_ * GetAllDecisionVars() - f_;
		Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(constraint.rows());
		Eigen::VectorXd inf_vec = CreateInfVector(constraint.rows());

		prog_.AddLinearConstraint(
				constraint, -inf_vec, zero_vec
				);
	}

	// **************** //
	// HELPER FUNCTIONS //
	// **************** //

	bool HoQpProblem::IsHigherPriProblemDefined()
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
			Eigen::MatrixXd m1, Eigen::MatrixXd m2
			)
	{
		assert (m1.cols() == m2.cols());
		Eigen::MatrixXd res(m1.rows() + m2.rows(), m1.cols());
		res << m1,
					 m2;

		return res;
	}

	Eigen::VectorXd HoQpProblem::ConcatenateVectors(
			Eigen::VectorXd v1, Eigen::VectorXd v2
			)
	{
		assert (v1.cols() == v2.cols());
		Eigen::VectorXd res(v1.rows() + v2.rows());
		res << v1,
					 v2;

		return res;
	}
}

