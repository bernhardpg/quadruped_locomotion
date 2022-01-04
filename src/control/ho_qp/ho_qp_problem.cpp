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

		ROS_INFO("Constructed new HoQpProblem");
		ROS_INFO("num_decision_vars_: %d", num_decision_vars_);
		ROS_INFO("num_slack_vars_: %d", num_slack_vars_);

		CreateDecisionVars();
		CreateSlackVars();

		AccumulateTasks();
		AccumulateSlackSolutions();
		SetAccumNullspacePrev();
		ConstructAccumNullspaceMatrix();

		ConstructDMatrix();
		ConstructFVector();
		ConstructHMatrix();
		ConstructCVector();

		AddIneqConstraints();
		AddQuadraticCost();

		SolveQp();

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
		return accum_tasks_.D.rows();	
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
		if (!IsHigherPriProblemDefined())
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
		if (!IsHigherPriProblemDefined())		
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

	void HoQpProblem::SetAccumNullspacePrev()
	{
		if (!IsHigherPriProblemDefined())
		{
			accum_Z_prev_ = Eigen::MatrixXd::Identity(
					num_decision_vars_, num_decision_vars_
					);
		}
		else
		{
			accum_Z_prev_ = higher_pri_problem_->GetAccumNullspaceMatrix();
		}
	}

	void HoQpProblem::ConstructAccumNullspaceMatrix()
	{
		if (!IsHigherPriProblemDefined())
		{
			accum_Z_ = CalcNullSpaceProjMatrix(curr_task_.A);
		}
		else
		{
			Eigen::MatrixXd Null_of_accum_A_curr_times_accum_Z_prev_ =
				CalcNullSpaceProjMatrix(accum_tasks_.A * accum_Z_prev_);

			accum_Z_ = accum_Z_prev_ * Null_of_accum_A_curr_times_accum_Z_prev_;
		}
	}

	void HoQpProblem::ConstructDMatrix()
	{
		int num_prev_slack_vars = GetPrevAccumNumSlackVars();

		Eigen::MatrixXd D(
				2 * num_slack_vars_ + num_prev_slack_vars,
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
					num_prev_slack_vars, num_slack_vars_
					);

		if (!IsHigherPriProblemDefined())
		{
			D << zero, -eye,
					 curr_task_.D, -eye;
		}
		else
		{
			//			TODO: remove
//			PrintMatrixSize("new D",D);
//			PrintMatrixSize("zero",zero);
//			PrintMatrixSize("eye",eye);
//			PrintMatrixSize("curr_task D",curr_task_.D);
//			PrintMatrixSize("accum D",higher_pri_problem_->GetAccumD());
//			PrintMatrixSize("Z_prev",higher_pri_problem_->GetAccumNullspaceMatrix());
//			PrintMatrixSize("zero acuum",accum_zero);

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
					 curr_task_.f; // Note: There is no previous solution here
		}
		else
		{
			// TODO: Remove these intermittent variables if it runs too slow
			Eigen::VectorXd accum_f_prev = higher_pri_problem_->GetAccumF();
			Eigen::VectorXd accum_D_prev_times_x_prev =
				higher_pri_problem_->GetAccumD()
				* higher_pri_problem_->GetSolution();
			Eigen::VectorXd accum_slack_vars_prev = 
				higher_pri_problem_->GetAccumSlackSolutions();
			Eigen::VectorXd D_curr_times_x_prev = curr_task_.D
				* higher_pri_problem_->GetSolution();

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

		Eigen::MatrixXd zero = Eigen::MatrixXd::Identity(
					num_slack_vars_, num_decision_vars_ 
					);
			
		Eigen::MatrixXd H(
				num_decision_vars_ + num_slack_vars_,
				num_decision_vars_ + num_slack_vars_);
		H.setZero();

		Eigen::MatrixXd A_curr_T_times_A_curr =
			curr_task_.A.transpose() * curr_task_.A;

		if (!IsHigherPriProblemDefined())
		{
			H << A_curr_T_times_A_curr, zero.transpose(),
					 zero, eye;
		}
		else
		{
			H << accum_Z_prev_.transpose() * A_curr_T_times_A_curr * accum_Z_prev_, zero.transpose(),
					 zero, eye;
		}

		//		TODO: remove
//		PrintMatrixSize("eye", eye);
//		PrintMatrixSize("zero", zero);
//		PrintMatrixSize("H", H);
//		PrintMatrixSize("A_T_A", A_curr_T_times_A_curr);

		H_ = H;
	}

	void HoQpProblem::ConstructCVector()
	{
		int num_prev_slack_vars = GetPrevAccumNumSlackVars();

		Eigen::VectorXd c(num_decision_vars_ + num_slack_vars_);
		c.setZero();

		Eigen::VectorXd zero_vec =
			Eigen::VectorXd::Zero(num_slack_vars_);

		if (!IsHigherPriProblemDefined())
		{
			c << curr_task_.A.transpose() * curr_task_.b,
					 zero_vec;
		}
		else
		{
			Eigen::VectorXd temp =
				accum_Z_prev_.transpose() * curr_task_.A.transpose()
				* (curr_task_.A * higher_pri_problem_->GetSolution()
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
		symbolic_vector_t constraint = D_ * GetAllDecisionVars() - f_;
		Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(constraint.rows());
		Eigen::VectorXd inf_vec = CreateInfVector(constraint.rows());

		prog_.AddLinearConstraint(
				constraint, -inf_vec, zero_vec
				);
	}

	void HoQpProblem::AddQuadraticCost()
	{
		symbolic_vector_t x = GetAllDecisionVars();
//		PrintMatrixSize("x", x);
//		PrintMatrixSize("H", H_);
//		PrintMatrixSize("c", c_);

		drake::symbolic::Expression cost = 
			x.transpose() * H_ * x
			+ (c_.transpose() * x)(0); 

		// H_ is known to be positive definite due to its structure
		bool is_convex = true;
		prog_.AddQuadraticCost(0.5 * cost, is_convex);
	}

	void HoQpProblem::SolveQp()
	{
		std::cout << "Solving QP" << std::endl;
		PrintTask(curr_task_);

		std::cout << "H:" << std::endl;
		PrintMatrix(H_);
		std::cout << "c:" << std::endl;
		PrintMatrix(c_);
		std::cout << "D:" << std::endl;
		PrintMatrix(D_);
		std::cout << "f:" << std::endl;
		PrintMatrix(f_);
		std::cout << prog_.to_string() << std::endl;

		result_ = Solve(prog_);
		ROS_INFO_STREAM("Solver id: " << result_.get_solver_id()
			<< "\nFound solution: " << result_.is_success()
			<< "\nSolution result: " << result_.get_solution_result()
			<< std::endl);
		assert(result_.is_success());
		std::cout << "Result: " << std::endl;
		PrintMatrix(result_.GetSolution());
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

