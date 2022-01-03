#include "control/hierarchical_qp.hpp"

// TODO: Remove all comments and print statements
// TODO: Clean up this class

namespace control
{
	HierarchicalQP::HierarchicalQP()
		: HierarchicalQP(3) // TODO: Default number of tasks
	{}

	HierarchicalQP::HierarchicalQP(int num_tasks)
		: num_tasks_(num_tasks),
			num_slack_vars_(num_tasks),
			quadratic_progs_(num_tasks_),
			u_dots_(num_tasks_),
			lambdas_(num_tasks_),
		  A_matrs_orig_(num_tasks_),
		  b_vecs_orig_(num_tasks_),
		  D_matrs_orig_(num_tasks_),
		  f_vecs_orig_(num_tasks_),
		  slack_vars_(num_tasks_),
		  slack_solutions_(num_tasks_),
		  slack_solutions_accum_(num_tasks_),
			A_matrs_accum_(num_tasks_),
			b_vecs_accum_(num_tasks_),
			D_matrs_accum_(num_tasks_),
			f_vecs_accum_(num_tasks_),
			Z_matrs_(num_tasks_),
			D_matrs_(num_tasks_),
			f_vecs_(num_tasks_)
	{
		num_contacts_ = 4; // TODO: generalize this

		num_decision_vars_ = kNumGenVels + kNumPosDims * num_contacts_;
		x_solution_.resize(num_decision_vars_); // TODO: move this?
		x_solution_.setZero();

		TestTasks();
		//SetupQPs();
	}

	// ******* //
	// TESTING //
	// ******* //

	// TODO: Just placeholder test code
	void HierarchicalQP::TestTasks()
	{
		// TODO: Testing
		Eigen::MatrixXd A1 = Eigen::MatrixXd::Random(1,num_decision_vars_);
		Eigen::VectorXd b1 = Eigen::VectorXd::Random(1);
		Eigen::MatrixXd D1 = Eigen::MatrixXd::Random(2,num_decision_vars_);
		Eigen::VectorXd f1 = Eigen::VectorXd::Random(2);

		TaskDefinition test_task = {A1, b1, D1, f1};
		HoQpProblem test_qp_problem = HoQpProblem();
		test_qp_problem.SetTask(test_task);

		Eigen::MatrixXd A2 = Eigen::MatrixXd::Random(3,num_decision_vars_);
		Eigen::VectorXd b2 = Eigen::VectorXd::Random(3);
		Eigen::MatrixXd D2 = Eigen::MatrixXd::Random(4,num_decision_vars_);
		Eigen::VectorXd f2 = Eigen::VectorXd::Random(4);

		TaskDefinition test_task_2 = {A2, b2, D2, f2};
		HoQpProblem test_qp_problem_2 = HoQpProblem();
		test_qp_problem_2.SetTask(test_task_2);
		test_qp_problem_2.SetHigherPriorityTask(
				test_qp_problem.GetAccumulatedTask()
				);

		Eigen::MatrixXd A3 = Eigen::MatrixXd::Random(5,num_decision_vars_);
		Eigen::VectorXd b3 = Eigen::VectorXd::Random(5);
		Eigen::MatrixXd D3 = Eigen::MatrixXd::Random(6,num_decision_vars_);
		Eigen::VectorXd f3 = Eigen::VectorXd::Random(6);

		TaskDefinition test_task_3 = {A3, b3, D3, f3};
		HoQpProblem test_qp_problem_3 = HoQpProblem();
		test_qp_problem_3.SetTask(test_task_3);
		test_qp_problem_3.SetHigherPriorityTask(
				test_qp_problem_2.GetAccumulatedTask()
				);

		TaskDefinition test = test_qp_problem_3.GetAccumulatedTask();
		PrintMatrix(test.A);
		PrintMatrix(test.b);
		PrintMatrix(test.D);
		PrintMatrix(test.f);
	}

	// ******************** //
	// OPTIMIZATION PROBLEM //
	// ******************** //
	
	void HierarchicalQP::SetupQPs()
	{
		// TODO: Add a processing function for eq and ineq matrices
		for (int task_i = 0; task_i < num_tasks_; ++task_i)
		{
			num_slack_vars_[task_i] = D_matrs_orig_[task_i].rows();
		}

		CreateEmptyMathProgs();

		AccumulateAMatrices();
		AccumulateBVectors();
		AccumulateDMatrices();
		AccumulateFVecs();
		std::cout << "Accumulated D matrices\n";

		ConstructNullSpaceMatrices();
		ConstructDMatrices();
		SetSlackVarsToZero(); // TODO Placeholder for solving the previous problems
		GetAccumulatedSlackSolutions();
		// These must be called AFTER solving the previous tasks!
		ConstructFVec(0);
		ConstructFVec(1);
		ConstructFVec(2);

		AddIneqConstraintsForTask(0);
		AddIneqConstraintsForTask(1);
		AddIneqConstraintsForTask(2);
	}

	void HierarchicalQP::SetSlackVarsToZero()
	{
		for (int task_i = 0; task_i < num_tasks_; ++task_i)	
		{
			slack_solutions_[task_i].resize(num_slack_vars_[task_i]);
			slack_solutions_[task_i].setZero();
		}
	}

	void HierarchicalQP::CreateEmptyMathProgs() // TODO:rename 
	{
		for (int task_i = 0; task_i < num_tasks_; ++task_i)
		{
			CreateNewMathProgForTask(task_i);
			CreateDecisionVariablesForTask(task_i);
		}
	}

	void HierarchicalQP::CreateNewMathProgForTask(int task_i)
	{
		std::unique_ptr<drake::solvers::MathematicalProgram>
			prog_ptr (new drake::solvers::MathematicalProgram);
		quadratic_progs_[task_i] = std::move(prog_ptr);
	}

	void HierarchicalQP::CreateDecisionVariablesForTask(int task_i)
	{
		u_dots_[task_i] =
			quadratic_progs_[task_i]
				->NewContinuousVariables(kNumGenVels, 1, "u_dot");
		lambdas_[task_i] =
			quadratic_progs_[task_i]
				->NewContinuousVariables(
						kNumPosDims * num_contacts_, 1, "lambda"
						);
	}

	void HierarchicalQP::CreateSlackVariablesForTask(
			int task_i, int num_slack_variables_for_task
			)
	{
		slack_vars_[task_i] =
			quadratic_progs_[task_i]
				->NewContinuousVariables(
						num_slack_variables_for_task, 1, "v"
						);
	}

	void HierarchicalQP::AddIneqConstraintsForTask(int task_i)
	{
		CreateSlackVariablesForTask(task_i, num_slack_vars_[task_i]);
		std::cout << "Adding ineq constraints for task " << task_i << std::endl;
		symbolic_vector_t decision_vars = GetAllDecisionVarsForTask(task_i);
		std::cout << decision_vars.transpose() << std::endl;

		PrintMatrixSize("D_matr", D_matrs_[task_i]);
		PrintMatrixSize("f_vec", f_vecs_[task_i]);
		PrintMatrixSize("decision_vars", decision_vars);
		symbolic_vector_t constraint =
			D_matrs_[task_i] * decision_vars - f_vecs_[task_i];
		Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(constraint.rows());
		Eigen::VectorXd inf_vec = CreateInfVector(constraint.rows());

		quadratic_progs_[task_i]->AddLinearConstraint(
				constraint, -inf_vec, zero_vec
				);
	}

	symbolic_vector_t HierarchicalQP::GetAllDecisionVarsForTask(int task_i)
	{
		int tot_num_decision_vars = u_dots_[task_i].rows()
			+ lambdas_[task_i].rows() + slack_vars_[task_i].rows();
		symbolic_vector_t decision_vars(tot_num_decision_vars);
		decision_vars << u_dots_[task_i],
										 lambdas_[task_i],
										 slack_vars_[task_i];
		return decision_vars;
	}

	// TODO: can I generealize these functions into one? it is essentially the exact same functionality 
	void HierarchicalQP::AccumulateAMatrices()
	{
		A_matrs_accum_[0] = A_matrs_orig_[0];
		int number_of_rows = A_matrs_accum_[0].rows();
		int number_of_cols = num_decision_vars_;

		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			number_of_rows += A_matrs_orig_[task_i].rows();	
			Eigen::MatrixXd curr_A(number_of_rows, number_of_cols);
			Eigen::MatrixXd prev_A = A_matrs_accum_[task_i - 1];
			curr_A << prev_A,
								A_matrs_orig_[task_i];
			A_matrs_accum_[task_i] = curr_A;
		}
	}

	void HierarchicalQP::AccumulateBVectors()
	{
		b_vecs_accum_[0] = b_vecs_orig_[0];
		int number_of_rows = b_vecs_accum_[0].rows();

		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			number_of_rows += b_vecs_orig_[task_i].rows();	
			Eigen::VectorXd curr_b(number_of_rows);
			Eigen::VectorXd prev_b = b_vecs_accum_[task_i - 1];
			curr_b << prev_b,
								b_vecs_orig_[task_i];
			b_vecs_accum_[task_i] = curr_b;
		}
	}

	void HierarchicalQP::AccumulateDMatrices()
	{
		D_matrs_accum_[0] = D_matrs_orig_[0];
		int number_of_rows = D_matrs_accum_[0].rows();
		int number_of_cols = num_decision_vars_;

		//PrintMatrix(D_matrs_accum_[0]);
		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			number_of_rows += D_matrs_orig_[task_i].rows();	
			Eigen::MatrixXd curr_D(number_of_rows, number_of_cols);
			Eigen::MatrixXd prev_D = D_matrs_accum_[task_i - 1];
			curr_D << prev_D,
								D_matrs_orig_[task_i];
			D_matrs_accum_[task_i] = curr_D;
			PrintMatrix(D_matrs_accum_[task_i]);
		}
	}

	void HierarchicalQP::AccumulateFVecs()
	{
		f_vecs_accum_[0] = f_vecs_orig_[0];
		int number_of_rows = f_vecs_accum_[0].rows();

		//PrintMatrix(f_vecs_accum_[0]);
		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			number_of_rows += f_vecs_orig_[task_i].rows();	
			Eigen::VectorXd curr_f(number_of_rows);
			Eigen::VectorXd prev_f = f_vecs_accum_[task_i - 1];
			curr_f << prev_f,
								f_vecs_orig_[task_i];
			f_vecs_accum_[task_i] = curr_f;
			PrintMatrix(f_vecs_accum_[task_i]);
		}
	}

	// TODO: This can be made more efficient by saving solutions between calls
	void HierarchicalQP::GetAccumulatedSlackSolutions()
	{
		std::cout << "Accumulating slack solutions\n";
		slack_solutions_accum_[0] = slack_solutions_[0];
		int number_of_rows = slack_solutions_accum_[0].rows();

		PrintMatrix(slack_solutions_accum_[0]);
		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			number_of_rows += slack_solutions_[task_i].rows();	
			Eigen::VectorXd curr_sol(number_of_rows);
			Eigen::VectorXd prev_sol = slack_solutions_accum_[task_i - 1];
			curr_sol << prev_sol,
								slack_solutions_[task_i];
			slack_solutions_accum_[task_i] = curr_sol;
			PrintMatrix(slack_solutions_accum_[task_i]);
		}
	}

	void HierarchicalQP::ConstructNullSpaceMatrices()
	{
		Z_matrs_[0] = CalcNullSpaceProjMatrix(A_matrs_accum_[0]);

		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			Z_matrs_[task_i] =
				ConstructNullSpaceMatrixFromPrevious(task_i);
		}
	}

	Eigen::MatrixXd HierarchicalQP::ConstructNullSpaceMatrixFromPrevious(
			int task_i
			)
	{
			Eigen::MatrixXd Z_prev = Z_matrs_[task_i - 1];
			Eigen::MatrixXd temp =
				CalcNullSpaceProjMatrix(A_matrs_accum_[task_i] * Z_prev);
			Eigen::MatrixXd Z_task_i = Z_prev * temp;
			return Z_task_i;
	}

	void HierarchicalQP::ConstructDMatrices()
	{
		for (int task_i = 0; task_i < num_tasks_; ++task_i)
		{
			ConstructDMatrix(task_i);
		}
	}

	void HierarchicalQP::ConstructDMatrix(int task_i)
	{
		// TODO: Put into its own function
		int total_num_slack_vars_so_far = 0;
		if (task_i > 0)
			total_num_slack_vars_so_far = D_matrs_accum_[task_i - 1].rows();

		Eigen::MatrixXd D(
				2 * num_slack_vars_[task_i] + total_num_slack_vars_so_far,
				num_decision_vars_ + num_slack_vars_[task_i]
				);
		Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(
					num_slack_vars_[task_i], num_slack_vars_[task_i]
					);
		Eigen::MatrixXd zero_for_curr_task = Eigen::MatrixXd::Zero(
					num_slack_vars_[task_i], num_decision_vars_
					);
		Eigen::MatrixXd zero_for_all_tasks = Eigen::MatrixXd::Zero(
					total_num_slack_vars_so_far, num_slack_vars_[task_i]
					);

		if (task_i == 0)
		{
			D << zero_for_curr_task, -eye,
					 D_matrs_orig_[task_i], -eye;
		}
		else
		{
//			PrintMatrixSize("zero_for_curr_task",zero_for_curr_task);
//			PrintMatrixSize("eye",eye);
//			PrintMatrixSize("D_matrix[i-1]",D_matrs_accum_[task_i - 1]);
//			PrintMatrixSize("Z_matrix[i-1]",Z_matrs_[task_i - 1]);
//			PrintMatrixSize("zero_for_all_tasks",zero_for_all_tasks);
//			PrintMatrixSize("task_ineq_const_D[i]",D_matrs_orig_[task_i]);
			D << zero_for_curr_task, -eye,
					 D_matrs_accum_[task_i - 1], zero_for_all_tasks,
					 D_matrs_orig_[task_i], - eye;
			// NOTE: This is upside down compared to the paper,
			// but more consistent with the rest of the algorithm
		}
		std::cout << "Constructed D matrix for " << task_i << std::endl;
		PrintMatrixSize("D",D);
		D_matrs_[task_i] = D;
		//PrintMatrix(D);
	}

	void HierarchicalQP::ConstructFVec(int task_i)
	{
		std::cout << "Constructing F vector for task " << task_i << std::endl;;
		// TODO: Put into its own function
		int total_num_slack_vars_so_far = 0;
		if (task_i > 0)
			total_num_slack_vars_so_far = D_matrs_accum_[task_i - 1].rows();

		Eigen::VectorXd f(
				2 * num_slack_vars_[task_i] + total_num_slack_vars_so_far
				);
		f.setZero();

		Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(
					num_slack_vars_[task_i]
					);

		auto f_curr = f_vecs_orig_[task_i];
		if (task_i == 0)
		{
			f << zero_vec,
					 f_curr;
		}
		else
		{
			PrintMatrixSize("D_matr", D_matrs_orig_[task_i]);
			PrintMatrixSize("x_sol", x_solution_);
			PrintMatrixSize("D_matr_accum", D_matrs_accum_[task_i - 1]);
			PrintMatrixSize("f_accum[task_i - 1]", f_vecs_accum_[task_i - 1]);
			PrintMatrixSize("f_vecs_orig_[task_i]", f_vecs_orig_[task_i]);
			PrintMatrixSize("slack_solutions_accum_[task_i - 1]", slack_solutions_accum_[task_i - 1]);

			auto fs_prev = f_vecs_accum_[task_i - 1];
			auto Ds_prev_times_x = D_matrs_accum_[task_i - 1] * x_solution_;
			auto vs_star_prev = slack_solutions_accum_[task_i - 1]; 
			auto D_curr_times_x = D_matrs_orig_[task_i] * x_solution_;

			f << zero_vec,
				   fs_prev - Ds_prev_times_x + vs_star_prev,
					 f_curr - D_curr_times_x;
		}
		PrintMatrix(f);

		f_vecs_[task_i] = f;
	}

	void HierarchicalQP::AddEqConstraint(
			Eigen::MatrixXd A,
			Eigen::VectorXd b,
			symbolic_vector_t decision_variables
			)
	{
//		symbolic_vector_t constraint = A * decision_variables - b;
//		auto squared_constraint = 0.5 * constraint.transpose() * constraint;
//		prog_.AddQuadraticCost(squared_constraint);
	}

	void HierarchicalQP::AddIneqConstraint(
			Eigen::MatrixXd D,
			Eigen::VectorXd f,
			symbolic_vector_t decision_variables
			)
	{
//		int num_rows = 	D.rows(); // TODO: is this a nice way to get number of rows?
//
//		symbolic_vector_t new_vs = prog_.NewContinuousVariables(num_rows, 1, "v");
//
//		symbolic_vector_t constraint = D * decision_variables - f - new_vs;
//		Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(num_rows);
//		Eigen::VectorXd inf_vec = CreateInfVector(num_rows);
//
//		prog_.AddLinearConstraint(constraint, -inf_vec, zero_vec);
		//prog_.AddQuadraticCost(0.5 * new_vs.transpose() * new_vs);
	}
}
