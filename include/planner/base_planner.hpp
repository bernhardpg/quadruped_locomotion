#pragma once

#include <Eigen/Core>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/mathematical_program_result.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/symbolic.h>

#include "variable_types.hpp"
#include "anymal_constants.hpp"

class BasePlanner
{
	public:
		BasePlanner(){};

		void PlanBaseMotion(
				const int polynomial_degree, const int n_traj_segments
				);
		void SetSupportPolygons(
				const std::vector<std::vector<Eigen::Vector2d>> &support_polygons
				);

		Eigen::VectorXd EvalBasePosAtT(const double t);
		Eigen::VectorXd EvalBaseVelAtT(const double t);
		Eigen::VectorXd EvalBaseAccAtT(const double t);

		const int GetNumTrajSegments();

	private:

		// ********** //
		// TRAJECTORY //
		// ********** //

		int polynomial_degree_;
		int n_traj_segments_;
		const int traj_dimension_ = k2D;

		Eigen::Vector2d pos_initial_;
		Eigen::Vector2d pos_final_;
		std::vector<std::vector<Eigen::Vector2d>> support_polygons_;

		polynomial_matrix_t polynomials_pos_;
		polynomial_matrix_t polynomials_vel_;
		polynomial_matrix_t polynomials_acc_;

		void GenerateTrajectory();
		void GeneratePolynomialsFromSolution();
		std::vector<symbolic_matrix_t> GetCoeffValues();
		polynomial_matrix_t GeneratePolynomials(
				const symbolic_vector_t &monomial_basis,
				const std::vector<symbolic_matrix_t> &coeff_values 
				);

		Eigen::VectorXd EvalBaseTrajAtT(
				const double t, const int derivative = 0
				);

		// ******************** //
		// OPTIMIZATION PROBLEM //
		// ******************** //

		drake::solvers::MathematicalProgram prog_;
		drake::solvers::MathematicalProgramResult result_;
		std::vector<symbolic_matrix_t> coeffs_;

		drake::symbolic::Variable t_;
		symbolic_vector_t m_; // monomial basis of t
		symbolic_vector_t m_dot_;
		symbolic_vector_t m_ddot_;
		symbolic_matrix_t T_ ; // Transformation matrices for m
		symbolic_matrix_t T_dot_;
		symbolic_matrix_t T_ddot_;

		void FormulateOptimizationProblem();
		void InitMonomials();
		void InitDecisionVariables();
		void AddAccelerationCost();
		void AddContinuityConstraints();
		void AddInitialAndFinalConstraint();

		void SolveOptimization();

		// **************** //
		// HELPER FUNCTIONS //
		// **************** //

		Eigen::MatrixXd GetTransformationMatrixAtT(
				double t, symbolic_vector_t v
				);
		symbolic_vector_t GetPosExpressionAtT(double t);
		symbolic_vector_t GetPosExpressionAtT(
				double t_in_segment, int segment_j
				);
		symbolic_vector_t GetVelExpressionAtT(
				double t_in_segment, int segment_j
				);
		symbolic_vector_t GetAccExpressionAtT(
				double t_in_segment, int segment_j
				);
		symbolic_vector_t GetTrajExpressionAtT(
				double t, symbolic_vector_t v, int segment_j
				);
		Eigen::Vector2d GetPolygonCentroid(
				const std::vector<Eigen::Vector2d> &polygon
				);
};

