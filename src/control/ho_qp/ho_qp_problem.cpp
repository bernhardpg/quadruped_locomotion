#include "control/ho_qp/ho_qp_problem.hpp"

// TODO: clean up use of variable 'x'

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
		InitTaskVariables();
		AccumulateTasks();

		// TODO remove
		ROS_INFO("----- Constructed new HoQpProblem -----");
		ROS_INFO("num_decision_vars_: %d", num_decision_vars_);
		ROS_INFO("num_slack_vars_: %d", num_slack_vars_);
		ROS_INFO("has eq constraints: %d", has_eq_constraints_);
		ROS_INFO("has ineq constraints: %d", has_ineq_constraints_);
		std::cout << "Current task:" << std::endl;
		PrintTask(curr_task_);

		SetPrevProblemValues();
		ROS_INFO("Stored values from prev problem");

		CreateDecisionVars();
		CreateSlackVars();
		ROS_INFO("Initialized variables");

		ConstructAccumNullspaceMatrix();
		ROS_INFO("Accumulated null space matrices");

		ConstructHMatrix();
		ROS_INFO("Constructed H matrix");
		ConstructCVector();
		ROS_INFO("Constructed c vector");
		ConstructDMatrix();
		ROS_INFO("Constructed D matrix");
		ConstructFVector();
		ROS_INFO("Constructed f vector");

		if (has_ineq_constraints_)
			AddIneqConstraints();
		AddQuadraticCost();

		SolveQp();
		AccumulateSlackSolutions();
		std::cout << "x: " << std::endl;
		PrintMatrix(GetSolution());

//		PrintMatrixSize("Z", accum_Z_);
//		PrintMatrixSize("H", H_);
//		PrintMatrixSize("c", c_);
//		PrintMatrixSize("D", D_);
//		PrintMatrixSize("f", f_);
	}

	// ***************** //
	// SETTERS & GETTERS //
	// ***************** //

	TaskDefinition HoQpProblem::GetAccumTasks()
	{
		return accum_tasks_;
	}

	Eigen::MatrixXd HoQpProblem::GetAccumA()
	{
		return accum_tasks_.A;
	}

	Eigen::MatrixXd HoQpProblem::GetAccumD()
	{
		return accum_tasks_.D;
	}

	Eigen::VectorXd HoQpProblem::GetAccumB()
	{
		return accum_tasks_.b;
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
//		std::cout << "accum_Z_prev_\n";
//		PrintMatrix(accum_Z_prev_);
//
//		std::cout << "x_prev\n";
//		PrintMatrix(x_prev_);
//
//		std::cout << "decision_vars_solutions_\n";
//		PrintMatrix(decision_vars_solutions_);
//
//		std::cout << "x\n";
//		PrintMatrix(x);
 
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

	symbolic_vector_t HoQpProblem::GetAllDecisionVars()
	{
		int tot_num_decision_vars =
			num_decision_vars_ + num_slack_vars_;
		symbolic_vector_t x(tot_num_decision_vars);
		x << decision_vars_, 
										 slack_vars_;
		return x;
	}


	// ********************* //
	// MATRIX INITIALIZATION //
	// ********************* //

	void HoQpProblem::AccumulateTasks()
	{
		if (!is_higher_pri_problem_defined_)
		{
			accum_tasks_ = curr_task_;
		}
		else
		{
			accum_tasks_ = ConcatenateTasks(
					curr_task_, higher_pri_problem_->GetAccumTasks()
					);
		}
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
		if (!is_higher_pri_problem_defined_)
			InitPrevProblemValuesToDefault();
		else
			InitPrevProblemValuesFromPrevProblem();
	}

	void HoQpProblem::InitPrevProblemValuesToDefault()
	{
		accum_Z_prev_ = Eigen::MatrixXd::Identity(
				num_decision_vars_, num_decision_vars_
				);
		x_prev_ = Eigen::VectorXd::Zero(num_decision_vars_);
		num_prev_slack_vars_ = 0;
	}

	void HoQpProblem::InitPrevProblemValuesFromPrevProblem()
	{
		accum_Z_prev_ = higher_pri_problem_->GetAccumNullspaceMatrix();
		x_prev_ = higher_pri_problem_->GetSolution();
		num_prev_slack_vars_ =
			higher_pri_problem_->GetAccumNumSlackVars();
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
		Eigen::MatrixXd Null_of_A_curr_times_accum_Z_prev_ =
			CalcNullSpaceProjMatrix(curr_task_.A * accum_Z_prev_);

		accum_Z_ = accum_Z_prev_ * Null_of_A_curr_times_accum_Z_prev_;
	}

	void HoQpProblem::ConstructDMatrix()
	{
		Eigen::MatrixXd D(
				2 * num_slack_vars_ + num_prev_slack_vars_,
				num_decision_vars_ + num_slack_vars_
				);
		D.setZero();

		Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(
					num_slack_vars_, num_slack_vars_
					);
		Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(
					num_slack_vars_, num_decision_vars_
					);
		Eigen::MatrixXd accum_zero = Eigen::MatrixXd::Zero(
					num_prev_slack_vars_, num_slack_vars_
					);

		if (!is_higher_pri_problem_defined_)
		{
			PrintMatrixSize("D",D);
			PrintMatrixSize("zero",zero);
			PrintMatrixSize("eye",eye);
			PrintMatrixSize("curr_task_.D",curr_task_.D);
			D << zero, -eye,
					 curr_task_.D, -eye;
		}
		else
		{
			std::cout << "here2\n";
			Eigen::MatrixXd accum_D_prev_times_accum_Z_prev_ = 
					 higher_pri_problem_->GetAccumD() * accum_Z_prev_;

			// NOTE: This is upside down compared to the paper,
			// but more consistent with the rest of the algorithm
			D << zero, -eye,
					 accum_D_prev_times_accum_Z_prev_, accum_zero,
					 curr_task_.D * accum_Z_prev_, - eye;
		}
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

		if (!is_higher_pri_problem_defined_)
		{
			f << zero_vec,
					 curr_task_.f; // Note: There is no previous solution here
		}
		else
		{
			// TODO: Remove these intermittent variables if it runs too slow
			Eigen::VectorXd accum_f_prev = higher_pri_problem_->GetAccumF();
			Eigen::VectorXd accum_D_prev_times_x_prev =
				higher_pri_problem_->GetAccumD()
				* x_prev_;
			Eigen::VectorXd accum_slack_vars_prev = 
				higher_pri_problem_->GetAccumSlackSolutions();
			Eigen::VectorXd D_curr_times_x_prev = curr_task_.D
				* x_prev_;

			//			TODO remove
//			PrintMatrixSize("prev f", prev_fs);
//			PrintMatrixSize("prev_Ds_times_x", prev_Ds_times_x);
//			PrintMatrixSize("prev_accum_slack_variables", prev_accum_slack_vars);
//			PrintMatrixSize("D_curr_times_x", D_curr_times_x);

			f << zero_vec,
				   accum_f_prev - accum_D_prev_times_x_prev + accum_slack_vars_prev,
					 curr_task_.f - D_curr_times_x_prev;
		}
		f_ = f;
	}

	void HoQpProblem::ConstructHMatrix()
	{
		Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(
					num_slack_vars_, num_slack_vars_
					);

		Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(
					num_slack_vars_, num_decision_vars_ 
					);
			
		Eigen::MatrixXd H(
				num_decision_vars_ + num_slack_vars_,
				num_decision_vars_ + num_slack_vars_);
		H.setZero();

		Eigen::MatrixXd Z_t_A_t_A_Z =
			Eigen::MatrixXd::Zero(num_decision_vars_, num_decision_vars_);

		if (has_eq_constraints_)
		{
			Eigen::MatrixXd A_t_A =
				curr_task_.A.transpose() * curr_task_.A;

			Z_t_A_t_A_Z = accum_Z_prev_.transpose()
				* A_t_A * accum_Z_prev_;
		}

		H << Z_t_A_t_A_Z, zero.transpose(),
				 zero, eye;

		//		TODO: remove
//		PrintMatrixSize("eye", eye);
//		PrintMatrixSize("zero", zero);
//		PrintMatrixSize("H", H);
//		PrintMatrixSize("A_T_A", A_curr_T_times_A_curr);

		H_ = H;
	}


	void HoQpProblem::ConstructCVector()
	{
		Eigen::VectorXd c(num_decision_vars_ + num_slack_vars_);
		c.setZero();

		Eigen::VectorXd zero_vec =
			Eigen::VectorXd::Zero(num_slack_vars_);

		if (!is_higher_pri_problem_defined_)
		{
			c << - curr_task_.A.transpose() * curr_task_.b,
					 zero_vec;
		}
		else
		{
			Eigen::VectorXd temp =
				accum_Z_prev_.transpose() * curr_task_.A.transpose()
				* (curr_task_.A * x_prev_
						- curr_task_.b);

			c << temp,
					 zero_vec;
		}

		c_ = c;
	}

	// ******************** //
	// OPTIMIZATION PROBLEM //
	// ******************** //

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
		prog_.AddLinearConstraint(
				D_ * GetAllDecisionVars() <= f_
				);
	}

	void HoQpProblem::AddQuadraticCost()
	{
		symbolic_vector_t x = GetAllDecisionVars();
//		PrintMatrixSize("x", x);
//		PrintMatrixSize("H", H_);
//		PrintMatrixSize("c", c_);

		drake::symbolic::Expression cost = 
			0.5 * x.transpose() * H_ * x
			+ (c_.transpose() * x)(0); 
	
//		std::cout << "quadratic cost " << std::endl;
//		std::cout << x.transpose() * H_ * x << std::endl;
//
//		std::cout << "linear cost " << std::endl;
//		std::cout << (c_.transpose() * x)(0) << std::endl;

		// H_ is known to be positive definite due to its structure
		bool is_convex = true;
		prog_.AddQuadraticCost(cost, is_convex);
	}

	void HoQpProblem::SolveQp()
	{
		std::cout << prog_.to_string() << std::endl;

		result_ = Solve(prog_);
		assert(result_.is_success());
		Eigen::VectorXd sol = result_.GetSolution();

		decision_vars_solutions_.resize(num_decision_vars_);
		decision_vars_solutions_ << sol
			.block(0,0,num_decision_vars_,1);
		slack_vars_solutions_.resize(num_slack_vars_);
		slack_vars_solutions_ << sol
			.block(num_decision_vars_,0,num_slack_vars_,1);

		std::cout << "H:" << std::endl;
		PrintMatrix(H_);
		std::cout << "c:" << std::endl;
		PrintMatrix(c_);
		std::cout << "D:" << std::endl;
		PrintMatrix(D_);
		std::cout << "f:" << std::endl;
		PrintMatrix(f_);
		ROS_INFO_STREAM("Solver id: " << result_.get_solver_id()
			<< "\nFound solution: " << result_.is_success()
			<< "\nSolution result: " << result_.get_solution_result()
			<< std::endl);
		std::cout << "Result: " << std::endl;
		std::cout << "z:\n";
		PrintMatrix(decision_vars_solutions_);
		std::cout << "v:\n";
		PrintMatrix(slack_vars_solutions_);

		GetSolution();
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
}

