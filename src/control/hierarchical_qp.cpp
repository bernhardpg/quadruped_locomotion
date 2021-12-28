#include "control/hierarchical_qp.hpp"

namespace control
{
	HierarchicalQP::HierarchicalQP()
		: HierarchicalQP(3)
	{}

	HierarchicalQP::HierarchicalQP(int num_tasks)
		: num_tasks_(num_tasks),
			quadratic_progs_(num_tasks),
			decision_variables_(num_tasks_),
		  eq_const_matrices_(num_tasks_),
		  eq_const_vectors_(num_tasks_),
		  ineq_const_matrices_(num_tasks_),
		  ineq_const_vectors_(num_tasks_),
		  slack_variables_(num_tasks_),
			accumulated_As(num_tasks_)
	{
		num_contacts_ = 4; // TODO: generalize this

		num_decision_vars_ = kNumGenVels + kNumPosDims * num_contacts_;
		PopulateVariables(); // TODO: Only for testing

		for (int task_i = 0; task_i < num_tasks_; ++task_i)
		{
			InitializeQPForTask(task_i);
		}

		CreateAccumulatedEqMatrices();
	}

	void HierarchicalQP::PopulateVariables()
	{
		// TODO: Just placeholder test code
		Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_decision_vars_, num_decision_vars_);
		eq_const_matrices_[0] = A;
		eq_const_matrices_[1] = A;
		eq_const_matrices_[2] = A;

		ineq_const_matrices_[0] = A;
		ineq_const_matrices_[1] = A;
		ineq_const_matrices_[2] = A;

		Eigen::MatrixXd b = Eigen::MatrixXd::Zero(num_decision_vars_,1);
		for (int i = 0; i < num_decision_vars_; ++i)
			b(i) = (double) i;

		eq_const_vectors_[0] = b;
		eq_const_vectors_[1] = b;
		eq_const_vectors_[2] = b;
		ineq_const_vectors_[0] = b;
		ineq_const_vectors_[1] = b;
		ineq_const_vectors_[2] = b;
	}

	void HierarchicalQP::InitializeQPForTask(int index)
	{
		std::unique_ptr<drake::solvers::MathematicalProgram>
			prog_ptr (new drake::solvers::MathematicalProgram);
		quadratic_progs_[index] = std::move(prog_ptr);
		decision_variables_[index] = CreateDecisionVariables(
				quadratic_progs_[index]
				);
		// TODO: 10 only for testing
		slack_variables_[index] = CreateSlackVariables(
				quadratic_progs_[index], 10
				);
		ROS_INFO("Created slack variables");
	}

	symbolic_vector_t HierarchicalQP::CreateDecisionVariables(
			std::unique_ptr<drake::solvers::MathematicalProgram> &prog
			)
	{
		symbolic_vector_t u_dot =
			prog->NewContinuousVariables(kNumGenVels, 1, "u_dot");
		symbolic_vector_t lambda =
			prog->NewContinuousVariables(
					kNumPosDims * num_contacts_, 1, "lambda"
					);
		symbolic_vector_t decision_vars(num_decision_vars_);
		decision_vars << u_dot, lambda;

		return decision_vars;
	}

	symbolic_vector_t HierarchicalQP::CreateSlackVariables(
			std::unique_ptr<drake::solvers::MathematicalProgram> &prog,
			int num_slack_variables
			)
	{
		symbolic_vector_t slack_variables =
			prog->NewContinuousVariables(
					num_slack_variables, 1, "v"
					);

		return slack_variables;
	}

	void HierarchicalQP::CreateAccumulatedEqMatrices()
	{
		accumulated_As[0] = eq_const_matrices_[0];
		int A_rows = eq_const_matrices_[0].rows();
		int A_cols = num_decision_vars_;
		for (int task_i = 1; task_i < num_tasks_; ++task_i)
		{
			A_rows += eq_const_matrices_[task_i].rows();	
			std::cout << "number of rows: " << A_rows << std::endl;
			Eigen::MatrixXd A_p(A_rows, A_cols);
			A_p << accumulated_As[task_i - 1],
						 eq_const_matrices_[task_i];
			accumulated_As[task_i] = A_p;

			std::cout << A_p << std::endl << std::endl;
		}
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

	Eigen::VectorXd HierarchicalQP::CreateInfVector(int size)
	{
		Eigen::VectorXd inf_vec = Eigen::VectorXd::Zero(size);
		for (int i = 0; i < size; ++i)
			inf_vec(i) = kInf;

		return inf_vec;
	}

	// TODO: delete
	void HierarchicalQP::Test()
	{
		// TEST
		Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_decision_vars_, num_decision_vars_);
		
		Eigen::MatrixXd b = Eigen::MatrixXd::Zero(num_decision_vars_,1);
		for (int i = 0; i < num_decision_vars_; ++i)
			b(i) = (double) i;
	}
}
