#include "control/hierarchical_qp.hpp"

namespace control
{
	HierarchicalQP::HierarchicalQP()
		: HierarchicalQP(3)
	{}

	HierarchicalQP::HierarchicalQP(int num_tasks)
		: num_tasks_(num_tasks),
			decision_variables_(num_tasks_),
			 eq_const_matrices_(num_tasks_),
			 eq_const_vectors_(num_tasks_),
			 ineq_const_matrices_(num_tasks_),
			 ineq_const_vectors_(num_tasks_),
			 slack_variables_(num_tasks_)
	{
		num_contacts_ = 4; // TODO: generalize this

		num_decision_vars_ = kNumGenVels + kNumPosDims * num_contacts_;
		PopulateVariables();
		CreateQPWithIndex(0);
		//CreateDecisionVariables();
	}

	void HierarchicalQP::CreateDecisionVariables()
	{
//		for (int task_i = 0; task_i < num_tasks_; ++task_i)
//		{
//			symbolic_vector_t u_dot = quadratic_progs_[task_i]
//				.NewContinuousVariables(kNumGenVels, 1, "u_dot");
//			symbolic_vector_t lambda = quadratic_progs_[task_i]
//				.NewContinuousVariables(kNumPosDims * num_contacts_, 1, "lambda");
//			symbolic_vector_t task_i_vars(num_decision_vars_);
//			task_i_vars << u_dot, lambda;
//		}
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

	void HierarchicalQP::CreateQPWithIndex(int index)
	{
		// TODO: How to make a new prog every time?
		drake::solvers::MathematicalProgram new_prog;

		symbolic_vector_t u_dot = new_prog
			.NewContinuousVariables(kNumGenVels, 1, "u_dot");
		symbolic_vector_t lambda = new_prog
			.NewContinuousVariables(kNumPosDims * num_contacts_, 1, "lambda");
		symbolic_vector_t task_i_vars(num_decision_vars_);
		task_i_vars << u_dot, lambda;
	}

	symbolic_vector_t HierarchicalQP::Create

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
