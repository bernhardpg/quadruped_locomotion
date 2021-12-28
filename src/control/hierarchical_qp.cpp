#include "control/hierarchical_qp.hpp"

namespace control
{
	HierarchicalQP::HierarchicalQP()
	{
		int num_contacts = 4; // TODO: generalize this

		u_dot_ = prog_.NewContinuousVariables(kNumGenVels, 1, "u_dot");
		lambda_ = prog_.
			NewContinuousVariables(kNumPosDims * num_contacts, 1, "lambda");
		num_decision_vars_ = u_dot_.rows() + lambda_.rows();

		// TEST
		Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_decision_vars_, num_decision_vars_);
		
		Eigen::MatrixXd b = Eigen::MatrixXd::Zero(num_decision_vars_,1);
		for (int i = 0; i < num_decision_vars_; ++i)
			b(i) = (double) i;

		//AddEqConstraint(A,b);
		AddIneqConstraint(A,b);
	}

	symbolic_vector_t HierarchicalQP::GetDecisionVars()
	{
		symbolic_vector_t decision_variables(num_decision_vars_);
		decision_variables << u_dot_, lambda_;
		return decision_variables;
	}

	void HierarchicalQP::AddEqConstraint(
			Eigen::MatrixXd A, Eigen::VectorXd b
			)
	{
		symbolic_vector_t constraint = A * GetDecisionVars() - b;
		auto squared_constraint = 0.5 * constraint.transpose() * constraint;
		prog_.AddQuadraticCost(squared_constraint);
	}

	void HierarchicalQP::AddIneqConstraint(
			Eigen::MatrixXd D, Eigen::VectorXd f
			)
	{
		int num_rows = 	D.rows(); // TODO: is this a nice way to get number of rows?

		symbolic_vector_t new_vs = prog_.NewContinuousVariables(num_rows, 1, "v");

		symbolic_vector_t constraint = D * GetDecisionVars() - f - new_vs;
		Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(num_rows);
		Eigen::VectorXd inf_vec = Eigen::VectorXd::Zero(num_rows);
		for (int i = 0; i < num_rows; ++i)
			inf_vec(i) = 999999;
		prog_.AddLinearConstraint(constraint, -inf_vec, zero_vec);
	}
}
