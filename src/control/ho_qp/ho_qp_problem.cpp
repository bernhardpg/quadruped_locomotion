#include "control/ho_qp/ho_qp_problem.hpp"

// TODO: clean up use of variable 'x'
// TODO: clean up this class and remove prints

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
		ros::Time time_begin = ros::Time::now();
		InitTaskVariables();
		ros::Duration duration = ros::Time::now() - time_begin;
		ROS_INFO("QP: Spent %lf secs on init task variables", duration.toSec());		

		time_begin = ros::Time::now();
		SetPrevProblemValues();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QP: Spent %lf secs on getting prev values", duration.toSec());		

		time_begin = ros::Time::now();
		FormulateOptimizationProblem();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QP: Spent %lf secs on formulating qp", duration.toSec());		
		time_begin = ros::Time::now();
		SolveQp();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QP: Spent %lf secs on solving qp", duration.toSec());		

		time_begin = ros::Time::now();
		AccumulateTasks();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QP: Spent %lf secs on accumulating tasks", duration.toSec());		

		time_begin = ros::Time::now();
		ConstructAccumNullspaceMatrix();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QP: Spent %lf secs on constructing nullspace", duration.toSec());		

		time_begin = ros::Time::now();
		AccumulateSlackSolutions();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QP: Spent %lf secs on accumulating slack vars", duration.toSec());		
	}

	// ***************** //
	// SETTERS & GETTERS //
	// ***************** //

	TaskDefinition HoQpProblem::GetAccumTasks()
	{
		return accum_tasks_;
	}

	Eigen::MatrixXd HoQpProblem::GetAccumD()
	{
		return accum_tasks_.D;
	}

	Eigen::VectorXd HoQpProblem::GetAccumF()
	{
		return accum_tasks_.f;
	}

	Eigen::MatrixXd HoQpProblem::GetAccumNullspaceMatrix()
	{
		return accum_Z_;
	}

	int HoQpProblem::GetAccumNumSlackVars()
	{
		return accum_tasks_.D.rows();	
	}

	Eigen::VectorXd HoQpProblem::GetSolution()
	{
		Eigen::MatrixXd x =
			x_prev_ + accum_Z_prev_ * decision_vars_solutions_;
		return x;
	}

	Eigen::VectorXd HoQpProblem::GetSlackSolutions()
	{
		return slack_vars_solutions_;
	}

	Eigen::VectorXd HoQpProblem::GetAccumSlackSolutions()
	{
		return accum_slack_vars_;
	}

	variable_vector_t HoQpProblem::GetAllDecisionVars()
	{
		int tot_num_decision_vars =
			num_decision_vars_ + num_slack_vars_;
		variable_vector_t all_decision_vars(tot_num_decision_vars);
		all_decision_vars << decision_vars_, 
										     slack_vars_;
		return all_decision_vars;
	}


	// ********************* //
	// MATRIX INITIALIZATION //
	// ********************* //

	void HoQpProblem::AccumulateTasks()
	{
		accum_tasks_ = ConcatenateTasks(curr_task_, accum_tasks_prev_);
	}

	void HoQpProblem::AccumulateSlackSolutions()
	{
		if (!is_higher_pri_problem_defined_)		
		{
			accum_slack_vars_	= GetSlackSolutions();
		}
		else
		{
			accum_slack_vars_	= ConcatenateVectors(
					higher_pri_problem_->GetAccumSlackSolutions(),
					GetSlackSolutions()
					);
		}
	}

	void HoQpProblem::SetPrevProblemValues()
	{
		if (is_higher_pri_problem_defined_)
			InitPrevProblemValuesFromPrevProblem();
		else
			InitPrevProblemValuesToDefault();
	}

	void HoQpProblem::InitPrevProblemValuesToDefault()
	{
		accum_tasks_prev_ = CreateEmptyTask(num_decision_vars_);
		accum_Z_prev_ = Eigen::MatrixXd::Identity(
				num_decision_vars_, num_decision_vars_
				);
		x_prev_ = Eigen::VectorXd::Zero(num_decision_vars_);
		num_prev_slack_vars_ = 0;
		accum_slack_solutions_prev_ = Eigen::VectorXd::Zero(0);
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
		accum_Z_prev_ = higher_pri_problem_->GetAccumNullspaceMatrix();
		accum_tasks_prev_ = higher_pri_problem_->GetAccumTasks();
		accum_slack_solutions_prev_ = 
			higher_pri_problem_->GetAccumSlackSolutions();
		x_prev_ = higher_pri_problem_->GetSolution();
		num_prev_slack_vars_ =
			higher_pri_problem_->GetAccumNumSlackVars();
	}

	void HoQpProblem::ConstructProblemMatrices()
	{
		ros::Time time_begin = ros::Time::now();
		ConstructHMatrix();
		ros::Duration duration = ros::Time::now() - time_begin;
		ROS_INFO("QPFormulateMat: Spent %lf secs on H matrix", duration.toSec());		

		time_begin = ros::Time::now();
		ConstructCVector();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QPFormulateMat: Spent %lf secs on c vector ", duration.toSec());		


		time_begin = ros::Time::now();
		ConstructDMatrix();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QPFormulateMat: Spent %lf secs on D matrix", duration.toSec());		


		time_begin = ros::Time::now();
		ConstructFVector();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QPFormulateMat: Spent %lf secs on f vector ", duration.toSec());		

	}

	void HoQpProblem::ConstructAccumNullspaceMatrix()
	{
		if(has_eq_constraints_)
			ConstructNullspaceMatrixFromPrev();
		else
			accum_Z_ = accum_Z_prev_;
	}

	void HoQpProblem::ConstructNullspaceMatrixFromPrev()
	{
		ros::Time time_begin = ros::Time::now();
		Eigen::MatrixXd Null_of_A_curr_times_accum_Z_prev_ =
			CalcNullSpaceProjMatrix(curr_task_.A * accum_Z_prev_);
		ros::Duration duration = ros::Time::now() - time_begin;
		ROS_INFO("Nullspace: Spent %lf secs on constructing nullspace", duration.toSec());		

		accum_Z_ = accum_Z_prev_ * Null_of_A_curr_times_accum_Z_prev_;
	}

	void HoQpProblem::ConstructDMatrix()
	{
		Eigen::MatrixXd D(
				2 * num_slack_vars_ + num_prev_slack_vars_,
				num_decision_vars_ + num_slack_vars_
				);
		D.setZero();

		Eigen::MatrixXd accum_zero = Eigen::MatrixXd::Zero(
					num_prev_slack_vars_, num_slack_vars_
					);

		Eigen::MatrixXd D_curr_Z;
		if (has_ineq_constraints_)
			D_curr_Z = curr_task_.D * accum_Z_prev_;
		else
			D_curr_Z = Eigen::MatrixXd::Zero(0,num_decision_vars_);

		// NOTE: This is upside down compared to the paper,
		// but more consistent with the rest of the algorithm
		D << zero_nv_nx_, -eye_nv_nv_,
				 accum_tasks_prev_.D * accum_Z_prev_, accum_zero,
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
				 accum_tasks_prev_.f - accum_tasks_prev_.D * x_prev_
					 + accum_slack_solutions_prev_,
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
			A_curr_Z_prev_ = curr_task_.A * accum_Z_prev_;
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
		ros::Time time_begin = ros::Time::now();
		ConstructProblemMatrices();
		ros::Duration duration = ros::Time::now() - time_begin;
		ROS_INFO("QPFormulate: Spent %lf secs on constructing prob matrices", duration.toSec());		

		time_begin = ros::Time::now();
		CreateDecisionVars();
		CreateSlackVars();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QPFormulate: Spent %lf secs on dec and slack vars", duration.toSec());		

		time_begin = ros::Time::now();
		if (has_ineq_constraints_)
			AddIneqConstraints();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QPFormulate: Spent %lf secs on adding ineq constraints", duration.toSec());		

		time_begin = ros::Time::now();
		AddQuadraticCost();
		duration = ros::Time::now() - time_begin;
		ROS_INFO("QPFormulate: Spent %lf secs on adding quad cost", duration.toSec());		
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

	void HoQpProblem::AddIneqConstraints()
	{
		auto inf_vector = kInf * Eigen::VectorXd::Ones(f_.rows());
		prog_.AddLinearConstraint(D_, -inf_vector, f_, GetAllDecisionVars());
	}

	void HoQpProblem::AddQuadraticCost()
	{
		auto all_decision_vars = GetAllDecisionVars();
		bool is_convex = true;

		ros::Time time_begin = ros::Time::now();
//		drake::symbolic::Expression cost = 
//			0.5 * all_decision_vars.transpose() * H_ * all_decision_vars
//			+ (c_.transpose() * all_decision_vars)(0); 
//	
//		// H_ is known to be positive definite due to its structure
//		prog_.AddQuadraticCost(cost, is_convex);
//		ros::Duration duration = ros::Time::now() - time_begin;
//		ROS_INFO("QPFormulate: Spent %lf secs on symbolic quad cost calc", duration.toSec());		
//
		// H_ is known to be positive definite due to its structure
		prog_.AddQuadraticCost(H_, c_, all_decision_vars, is_convex);
		ros::Duration duration = ros::Time::now() - time_begin;
		ROS_INFO("QPFormulate: Spent %lf secs on alternative quad cost", duration.toSec());		
	}

	void HoQpProblem::SolveQp()
	{
		result_ = Solve(prog_);
//		ROS_INFO_STREAM("Solver id: " << result_.get_solver_id()
//			<< "\nFound solution: " << result_.is_success()
//			<< "\nSolution result: " << result_.get_solution_result()
//			<< std::endl);

		assert(result_.is_success());
		Eigen::VectorXd sol = result_.GetSolution();

		decision_vars_solutions_.resize(num_decision_vars_);
		decision_vars_solutions_ << sol
			.block(0,0,num_decision_vars_,1);
		slack_vars_solutions_.resize(num_slack_vars_);
		slack_vars_solutions_ << sol
			.block(num_decision_vars_,0,num_slack_vars_,1);
	}

	// **************** //
	// HELPER FUNCTIONS //
	// **************** //

	void HoQpProblem::InitTaskVariables()
	{
		num_decision_vars_ =
			std::max(curr_task_.A.cols(), curr_task_.D.cols());
		num_slack_vars_ = curr_task_.D.rows();
		has_eq_constraints_ = curr_task_.A.rows() > 0;
		has_ineq_constraints_ = num_slack_vars_ > 0;
		is_higher_pri_problem_defined_ = higher_pri_problem_ != nullptr;

		eps_matrix_ = eps_ * Eigen::MatrixXd::Identity(num_decision_vars_,num_decision_vars_);
		eye_nv_nv_ = Eigen::MatrixXd::Identity(
					num_slack_vars_, num_slack_vars_
					);
		zero_nv_nx_ = Eigen::MatrixXd::Zero(
					num_slack_vars_, num_decision_vars_
					);
	}
}

