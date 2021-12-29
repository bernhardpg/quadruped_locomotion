#include "control/hierarchical_qp.hpp"

namespace control
{
	HierarchicalQP::HierarchicalQP()
		: HierarchicalQP(3)
	{}

	HierarchicalQP::HierarchicalQP(int num_tasks)
		: num_tasks_(num_tasks),
			quadratic_progs_(num_tasks),
			u_dots_(num_tasks_),
			lambdas_(num_tasks_),
		  task_eq_const_As_(num_tasks_),
		  task_eq_const_bs_(num_tasks_),
		  task_ineq_const_Ds_(num_tasks_),
		  task_ineq_const_fs_(num_tasks_),
		  task_ineq_slack_variables_(num_tasks_),
			A_matrices_(num_tasks_),
			b_vectors_(num_tasks_),
			D_matrices_(num_tasks_),
			f_vectors_(num_tasks_),
			Z_matrices_(num_tasks_)
	{
		num_contacts_ = 4; // TODO: generalize this

		num_decision_vars_ = kNumGenVels + kNumPosDims * num_contacts_;

		PopulateTestVariables(); // TODO: Only for testing
		SetupQPs();
	}

	// ******* //
	// TESTING //
	// ******* //

	void HierarchicalQP::PopulateTestVariables()
	{
		// TODO: Just placeholder test code
		Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_decision_vars_, num_decision_vars_);

		Eigen::MatrixXd b = Eigen::MatrixXd::Zero(num_decision_vars_,1);
		for (int i = 0; i < num_decision_vars_; ++i)
			b(i) = (double) i;

		task_eq_const_As_[0] = A;
		task_eq_const_bs_[0] = b;

		task_eq_const_As_[1] = A;
		task_eq_const_bs_[1] = b;

		task_eq_const_As_[2] = A;
		task_eq_const_bs_[2] = b;

		task_ineq_const_Ds_[0] = A;
		task_ineq_const_fs_[0] = b;

		task_ineq_const_Ds_[1] = A;
		task_ineq_const_fs_[1] = b;

		task_ineq_const_Ds_[2] = A;
		task_ineq_const_fs_[2] = b;
	}

	// ******************** //
	// OPTIMIZATION PROBLEM //
	// ******************** //
	
	void HierarchicalQP::SetupQPs()
	{
		CreateEmptyMathProgs();

		AccumulateAMatrices();
		AccumulateBVectors();
		AccumulateDMatrices();
		std::cout << "Accumulated D matrices\n";

		ConstructNullSpaceMatrices();
		ConstructDMatrices();
	}

	void HierarchicalQP::CreateEmptyMathProgs() // TODO: renamce
	{
		for (int task_i = 0; task_i < num_tasks_; ++task_i)
		{
			CreateNewMathProgForTask(task_i);
			CreateDecisionVariablesForTask(task_i);
			CreateSlackVariablesForTask(task_i); 
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
			int task_i
			)
	{
		task_ineq_slack_variables_[task_i] =
			quadratic_progs_[task_i]
				->NewContinuousVariables(
						num_decision_vars_, 1, "v"
						);
	}

	// TODO: can I generealize these functions into one? it is essentially the exact same functionality 
	void HierarchicalQP::AccumulateAMatrices()
	{
		A_matrices_[0] = task_eq_const_As_[0];
		int number_of_rows = A_matrices_[0].rows();
		int number_of_cols = num_decision_vars_;

		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			number_of_rows += task_eq_const_As_[task_i].rows();	
			Eigen::MatrixXd curr_A(number_of_rows, number_of_cols);
			Eigen::MatrixXd prev_A = A_matrices_[task_i - 1];
			curr_A << prev_A,
								task_eq_const_As_[task_i];
			A_matrices_[task_i] = curr_A;
		}
	}

	void HierarchicalQP::AccumulateBVectors()
	{
		b_vectors_[0] = task_eq_const_bs_[0];
		int number_of_rows = b_vectors_[0].rows();

		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			number_of_rows += task_eq_const_bs_[task_i].rows();	
			Eigen::VectorXd curr_b(number_of_rows);
			Eigen::VectorXd prev_b = b_vectors_[task_i - 1];
			curr_b << prev_b,
								task_eq_const_bs_[task_i];
			b_vectors_[task_i] = curr_b;
		}
	}

	void HierarchicalQP::AccumulateDMatrices()
	{
		D_matrices_[0] = task_ineq_const_Ds_[0];
		int number_of_rows = D_matrices_[0].rows();
		int number_of_cols = num_decision_vars_;

		std::cout << D_matrices_[0] << std::endl << std::endl;
		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			number_of_rows += task_ineq_const_Ds_[task_i].rows();	
			Eigen::MatrixXd curr_D(number_of_rows, number_of_cols);
			Eigen::MatrixXd prev_D = D_matrices_[task_i - 1];
			curr_D << prev_D,
								task_ineq_const_Ds_[task_i];
			D_matrices_[task_i] = curr_D;
			std::cout << D_matrices_[task_i] << std::endl << std::endl;
		}
	}

	void HierarchicalQP::ConstructNullSpaceMatrices()
	{
		Z_matrices_[0] = CalcNullSpaceProjMatrix(A_matrices_[0]);

		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			Z_matrices_[task_i] =
				ConstructNullSpaceMatrixFromPrevious(task_i);
		}
	}

	Eigen::MatrixXd HierarchicalQP::ConstructNullSpaceMatrixFromPrevious(
			int task_i
			)
	{
			Eigen::MatrixXd Z_prev = Z_matrices_[task_i - 1];
			Eigen::MatrixXd temp =
				CalcNullSpaceProjMatrix(A_matrices_[task_i] * Z_prev);
			Eigen::MatrixXd Z_task_i = Z_prev * temp;
			return Z_task_i;
	}

	void HierarchicalQP::ConstructDMatrices()
	{
		ConstructDMatrix(0);
		ConstructDMatrix(1);
	}

	void HierarchicalQP::ConstructDMatrix(int task_i)
	{
		int num_slack_vars_for_task = task_ineq_const_Ds_[task_i].rows();
		int total_num_slack_vars_so_far;
		if (task_i == 0)
		{
			total_num_slack_vars_so_far = 0;
		}
		else
		{
			total_num_slack_vars_so_far = D_matrices_[task_i - 1].rows();
		}
		Eigen::MatrixXd D(
				2 * num_slack_vars_for_task + total_num_slack_vars_so_far,
				num_decision_vars_ + num_slack_vars_for_task 
				);
		Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(
					num_slack_vars_for_task, num_slack_vars_for_task 
					);
		Eigen::MatrixXd zero_for_curr_task = Eigen::MatrixXd::Zero(
					num_slack_vars_for_task, num_decision_vars_
					);
		Eigen::MatrixXd zero_for_all_tasks = Eigen::MatrixXd::Zero(
					num_slack_vars_for_task, num_decision_vars_
					);

		if (task_i == 0)
		{
			D << zero_for_curr_task, -eye,
					 task_ineq_const_Ds_[task_i], -eye;
		}
		else
		{
			D << zero_for_curr_task, -eye,
					 D_matrices_[task_i - 1] * Z_matrices_[task_i - 1], zero_for_all_tasks,
					 task_ineq_const_Ds_[task_i] * Z_matrices_[task_i - 1], - eye;
		}
		std::cout << D << std::endl << std::endl;

//		std::cout << "D rows: " << D.rows() << "cols: " << D.cols() << std:: endl; 
//
//		std::cout << "D_matrix[i - 1] rows: " << D_matrices_[task_i - 1].rows() 
//			<< "cols: " << D_matrices_[task_i - 1].cols() <<
//			std:: endl; 
//
//		std::cout << "Z_matrix[i-1] rows: " << Z_matrices_[task_i - 1].rows() 
//			<< "cols: " << Z_matrices_[task_i - 1].cols() <<
//			std:: endl; 
//
//		std::cout << "task_ineq_const_D[i] rows: " << task_ineq_const_Ds_[task_i].rows() 
//			<< "cols: " << task_ineq_const_Ds_[task_i].cols() <<
//			std:: endl; 
//
//		std::cout << D_matrices_[task_i - 1] * Z_matrices_[task_i - 1] << std::endl << std::endl;
//
//		std::cout << task_ineq_const_Ds_[task_i - 1] * Z_matrices_[task_i - 1] << std::endl << std::endl;
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


	// **************** //
	// HELPER FUNCTIONS //
	// **************** //

	Eigen::VectorXd HierarchicalQP::CreateInfVector(int size)
	{
		Eigen::VectorXd inf_vec = Eigen::VectorXd::Zero(size);
		for (int i = 0; i < size; ++i)
			inf_vec(i) = kInf;

		return inf_vec;
	}

	// TODO: Now this is used two places, so this should be its own library
	Eigen::MatrixXd HierarchicalQP::CalcNullSpaceProjMatrix(Eigen::MatrixXd A)
	{
		Eigen::MatrixXd A_inv = CalcPseudoInverse(A);
		Eigen::MatrixXd eye =
			Eigen::MatrixXd::Identity(A.cols(), A.cols());

		Eigen::MatrixXd null_space_projection_matrix = eye - A_inv * A;
		return null_space_projection_matrix;
	}

	Eigen::MatrixXd HierarchicalQP::CalcPseudoInverse(Eigen::MatrixXd A)
	{
		// Moore-Penrose right inverse: A^t (A A^t)
		Eigen:: MatrixXd pseudo_inverse =
			A.transpose() * (A * A.transpose()).inverse();
		return pseudo_inverse;
	}
}
