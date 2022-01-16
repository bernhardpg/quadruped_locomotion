#include "control/ho_qp/ho_qp_problem.hpp"

// TODO: clean up use of variable 'x'
namespace control
{
	HoQpProblem::HoQpProblem(TaskDefinition &new_task)
		: HoQpProblem(new_task, std::shared_ptr<HoQpProblem>(nullptr))
	{
	}

	HoQpProblem::HoQpProblem(
			TaskDefinition &new_task,
			std::shared_ptr<HoQpProblem> higher_pri_problem
			)
		: curr_task_(new_task), higher_pri_problem_(higher_pri_problem)
	{
		is_higher_pri_problem_defined_ = higher_pri_problem_ != nullptr;
		LoadPrevProblemData();
		InitTaskVariables();
		StackTasks();
		FormulateOptimizationProblem();
		SolveQp();
		ConstructStackedNullspaceMatrix();
		StackSlackSolutions();
	}

	// ***************** //
	// SETTERS & GETTERS //
	// ***************** //

	TaskDefinition HoQpProblem::GetStackedTasks()
	{
		return stacked_tasks_;
	}

	Eigen::MatrixXd HoQpProblem::GetStackedNullspaceMatrix()
	{
		return stacked_Z_;
	}

	int HoQpProblem::GetStackedNumSlackVars()
	{
		return stacked_tasks_.D.rows();	
	}

	Eigen::VectorXd HoQpProblem::GetSolution()
	{
		Eigen::MatrixXd x =
			x_prev_ + stacked_Z_prev_ * decision_vars_solutions_;
		return x;
	}

	Eigen::VectorXd HoQpProblem::GetSlackSolutions()
	{
		return slack_vars_solutions_;
	}

	Eigen::VectorXd HoQpProblem::GetStackedSlackSolutions()
	{
		return stacked_slack_vars_;
	}


	// ********************* //
	// MATRIX INITIALIZATION //
	// ********************* //

	void HoQpProblem::InitTaskVariables()
	{
		num_slack_vars_ = curr_task_.D.rows();
		has_eq_constraints_ = curr_task_.A.rows() > 0;
		has_ineq_constraints_ = num_slack_vars_ > 0;

		eps_matrix_ = eps_ * Eigen::MatrixXd::Identity(num_decision_vars_,num_decision_vars_);
		eye_nv_nv_ = Eigen::MatrixXd::Identity(
					num_slack_vars_, num_slack_vars_
					);
		zero_nv_nx_ = Eigen::MatrixXd::Zero(
					num_slack_vars_, num_decision_vars_
					);
	}

	void HoQpProblem::StackTasks()
	{
		stacked_tasks_ = ConcatenateTasks(curr_task_, stacked_tasks_prev_);
	}

	void HoQpProblem::StackSlackSolutions()
	{
		if (!is_higher_pri_problem_defined_)		
		{
			stacked_slack_vars_	= GetSlackSolutions();
		}
		else
		{
			stacked_slack_vars_	= ConcatenateVectors(
					higher_pri_problem_->GetStackedSlackSolutions(),
					GetSlackSolutions()
					);
		}
	}

	void HoQpProblem::LoadPrevProblemData()
	{
		if (is_higher_pri_problem_defined_)
		{
			InitPrevProblemValuesFromPrevProblem();
			num_decision_vars_ = stacked_Z_prev_.cols();
		}
		else
		{
			num_decision_vars_ = std::max(
					curr_task_.A.cols(), curr_task_.D.cols()
					);
			InitPrevProblemValuesToDefault();
		}
	}

	void HoQpProblem::InitPrevProblemValuesToDefault()
	{
		stacked_tasks_prev_ = CreateEmptyTask(num_decision_vars_);
		stacked_Z_prev_ = Eigen::MatrixXd::Identity(
				num_decision_vars_, num_decision_vars_
				);
		x_prev_ = Eigen::VectorXd::Zero(num_decision_vars_);
		num_prev_slack_vars_ = 0;
		stacked_slack_solutions_prev_ = Eigen::VectorXd::Zero(0);
	}

	TaskDefinition HoQpProblem::CreateEmptyTask(int num_decision_vars)
	{
		Eigen::MatrixXd A
			= Eigen::MatrixXd::Zero(0, num_decision_vars_);
		Eigen::VectorXd b
			= Eigen::VectorXd::Zero(0);
		Eigen::MatrixXd D
			= Eigen::MatrixXd::Zero(0, num_decision_vars_);
		Eigen::VectorXd f
			= Eigen::VectorXd::Zero(0);

		TaskDefinition empty_task = {A, b, D, f};
		return empty_task;
	}

	void HoQpProblem::InitPrevProblemValuesFromPrevProblem()
	{
		stacked_Z_prev_ = higher_pri_problem_->GetStackedNullspaceMatrix();
		stacked_tasks_prev_ = higher_pri_problem_->GetStackedTasks();
		stacked_slack_solutions_prev_ = 
			higher_pri_problem_->GetStackedSlackSolutions();
		x_prev_ = higher_pri_problem_->GetSolution();
		num_prev_slack_vars_ =
			higher_pri_problem_->GetStackedNumSlackVars();
	}

	void HoQpProblem::ConstructProblemMatrices()
	{
		ConstructHMatrix();
		ConstructCVector();
		ConstructDMatrix();
		ConstructFVector();
	}

	void HoQpProblem::ConstructStackedNullspaceMatrix()
	{
		if(has_eq_constraints_)
			ConstructNullspaceMatrixFromPrev();
		else
			stacked_Z_ = stacked_Z_prev_;
	}

	void HoQpProblem::ConstructNullspaceMatrixFromPrev()
	{
		Eigen::MatrixXd Null_of_A_curr_times_stacked_Z_prev_ =
			CalcNullSpaceProjMatrix(curr_task_.A * stacked_Z_prev_);

		stacked_Z_ = stacked_Z_prev_ * Null_of_A_curr_times_stacked_Z_prev_;
	}

	void HoQpProblem::ConstructDMatrix()
	{
		Eigen::MatrixXd D(
				2 * num_slack_vars_ + num_prev_slack_vars_,
				num_decision_vars_ + num_slack_vars_
				);
		D.setZero();

		Eigen::MatrixXd stacked_zero = Eigen::MatrixXd::Zero(
					num_prev_slack_vars_, num_slack_vars_
					);

		Eigen::MatrixXd D_curr_Z;
		if (has_ineq_constraints_)
			D_curr_Z = curr_task_.D * stacked_Z_prev_;
		else
			D_curr_Z = Eigen::MatrixXd::Zero(0,num_decision_vars_);

		// NOTE: This is upside down compared to the paper,
		// but more consistent with the rest of the algorithm
		D << zero_nv_nx_, -eye_nv_nv_,
				 stacked_tasks_prev_.D * stacked_Z_prev_, stacked_zero,
				 D_curr_Z, - eye_nv_nv_;

		D_ = D;
	}

	void HoQpProblem::ConstructFVector()
	{
		Eigen::VectorXd f(
				2 * num_slack_vars_ + num_prev_slack_vars_ 
				);
		f.setZero();

		Eigen::VectorXd zero_vec =
			Eigen::VectorXd::Zero(num_slack_vars_);

		Eigen::VectorXd f_minus_D_x_prev;
		if (has_ineq_constraints_)
			f_minus_D_x_prev = curr_task_.f - curr_task_.D * x_prev_;
		else
			f_minus_D_x_prev = Eigen::VectorXd::Zero(0);

		f << zero_vec,
				 stacked_tasks_prev_.f - stacked_tasks_prev_.D * x_prev_
					 + stacked_slack_solutions_prev_,
				 f_minus_D_x_prev;

		f_ = f;
	}

	void HoQpProblem::ConstructHMatrix()
	{
		Eigen::MatrixXd H(
				num_decision_vars_ + num_slack_vars_,
				num_decision_vars_ + num_slack_vars_);
		H.setZero();

		Eigen::MatrixXd Z_t_A_t_A_Z(num_decision_vars_,num_decision_vars_);
		if (has_eq_constraints_)
		{
			// Make sure that all eigenvalues of A_t_A are nonnegative,
			// which could arise due to numerical issues
			A_curr_Z_prev_ = curr_task_.A * stacked_Z_prev_;
			Z_t_A_t_A_Z =
				A_curr_Z_prev_.transpose() * A_curr_Z_prev_ 
				+ eps_matrix_;
			// This way of splitting up the multiplication is about twice as fast as multiplying 4 matrices
		}
		else
		{
			Z_t_A_t_A_Z.setZero();
		}

		H << Z_t_A_t_A_Z, zero_nv_nx_.transpose(),
				 zero_nv_nx_, eye_nv_nv_;

		H_ = H;
	}


	void HoQpProblem::ConstructCVector()
	{
		Eigen::VectorXd c(num_decision_vars_ + num_slack_vars_);
		c.setZero();

		Eigen::VectorXd zero_vec =
			Eigen::VectorXd::Zero(num_slack_vars_);

		Eigen::VectorXd temp(num_decision_vars_, 1);
		if (has_eq_constraints_)
		{
			temp = A_curr_Z_prev_.transpose()
			* (curr_task_.A * x_prev_
					- curr_task_.b);
		}
		else
		{
			temp.setZero();
		}

		c << temp,
				 zero_vec;

		c_ = c;
	}

	// ******************** //
	// OPTIMIZATION PROBLEM //
	// ******************** //

	void HoQpProblem::FormulateOptimizationProblem()
	{
		ConstructProblemMatrices();
		CreateDecisionVars();
		CreateSlackVars();
		SetAllDecisionVars();
		if (has_ineq_constraints_)
			AddIneqConstraints();
		AddQuadraticCost();
	}

	void HoQpProblem::CreateDecisionVars()
	{
		decision_vars_ = prog_.NewContinuousVariables(
						num_decision_vars_, 1, "z"
						);
	}

	void HoQpProblem::CreateSlackVars()
	{
		slack_vars_ = prog_.NewContinuousVariables(
						num_slack_vars_, 1, "v"
						);
	}

	void HoQpProblem::SetAllDecisionVars()
	{
		all_decision_vars_ = variable_vector_t(
				num_decision_vars_ + num_slack_vars_
				);
		all_decision_vars_ << decision_vars_, 
										      slack_vars_;
	}

	void HoQpProblem::AddIneqConstraints()
	{
		auto inf_vector = kInf * Eigen::VectorXd::Ones(f_.rows());
		prog_.AddLinearConstraint(D_, -inf_vector, f_, all_decision_vars_); 
	}

	void HoQpProblem::AddQuadraticCost()
	{
		// H_ is known to be positive definite due to its structure
		prog_.AddQuadraticCost(H_, c_, all_decision_vars_, true);
	}

	void HoQpProblem::SolveQp()
	{
		result_ = Solve(prog_);
		if (print_solver_details_)
		{
			ROS_INFO_STREAM("Solver id: " << result_.get_solver_id()
				<< "\nFound solution: " << result_.is_success()
				<< "\nSolution result: " << result_.get_solution_result()
				<< std::endl);
		}

		assert(result_.is_success());
		Eigen::VectorXd sol = result_.GetSolution();

		decision_vars_solutions_.resize(num_decision_vars_);
		decision_vars_solutions_ << sol
			.block(0,0,num_decision_vars_,1);
		slack_vars_solutions_.resize(num_slack_vars_);
		slack_vars_solutions_ << sol
			.block(num_decision_vars_,0,num_slack_vars_,1);
	}
}

