#pragma once

#include <Eigen/Core>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/mathematical_program_result.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/symbolic.h>

#include "variable_types.hpp"
#include "anymal_constants.hpp"
#include "planner/gait_sequence.hpp"

class BasePlanner
{
	public:
		BasePlanner();

		// *************** //
		// WALK TRAJECTORY //
		// *************** //

		void PlanBaseWalkMotion(const double target_height);
		void SetGaitSequence(GaitSequence gait_sequence);
		void SetSupportPolygons(
				const std::vector<std::vector<Eigen::Vector2d>> &support_polygons
				);
		void SetCurrPos(const Eigen::Vector2d &curr_pos);

		Eigen::VectorXd EvalWalkTrajPosAtT(const double t);
		Eigen::VectorXd EvalWalkTrajVelAtT(const double t);
		Eigen::VectorXd EvalWalkTrajAccAtT(const double t);
		const int GetNumTrajSegments();
		const double GetSecondsPerSegment();

		// ******* //
		// STANDUP //
		// ******* //

		void PlanBaseStandupMotion(
				const double seconds_to_standup_config,
				const double target_height, 
				const Eigen::MatrixXd &curr_pose
				);

		Eigen::VectorXd EvalStandupPosTrajAtT(const double time);
		Eigen::VectorXd EvalStandupVelTrajAtT(const double time);
		Eigen::VectorXd EvalStandupAccTrajAtT(const double time);
	

	private:

		// ******* //
		// STANDUP //
		// ******* //

		double seconds_to_standup_;

		drake::trajectories::PiecewisePolynomial<double>
			standup_traj_pos_;
		drake::trajectories::PiecewisePolynomial<double>
			standup_traj_vel_;
		drake::trajectories::PiecewisePolynomial<double>
			standup_traj_acc_;

		// *************** //
		// WALK TRAJECTORY //
		// *************** //

		int polynomial_degree_;
		int n_traj_segments_;
		double seconds_per_segment_;
		const int traj_dimension_ = k2D;
		double walking_height_;
		GaitSequence gait_sequence_;

		Eigen::Vector2d curr_2d_pos_;
		std::vector<std::vector<Eigen::Vector2d>> support_polygons_;

		polynomial_matrix_t polynomials_pos_;
		polynomial_matrix_t polynomials_vel_;
		polynomial_matrix_t polynomials_acc_;

		void GenerateWalkTrajectory();
		void GeneratePolynomialsFromSolution();
		std::vector<symbolic_matrix_t> GetCoeffValues();
		polynomial_matrix_t GeneratePolynomials(
				const symbolic_vector_t &monomial_basis,
				const std::vector<symbolic_matrix_t> &coeff_values 
				);

		Eigen::VectorXd EvalWalkTrajAtT(
				const double t, const int derivative = 0
				);

		// ******************** //
		// OPTIMIZATION PROBLEM //
		// ******************** //

		std::unique_ptr<drake::solvers::MathematicalProgram> prog_;
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
		void CreateNewMathProg();
		void InitMonomials();
		void InitDecisionVariables();
		void AddAccelerationCost();
		void AddContinuityConstraints();
		void AddInitialPosConstraint();
		void AddFinalPosConstraint();
		void AddZmpConstraints();

		std::vector<Eigen::MatrixXd> polygons_A_matrices_;
		std::vector<Eigen::VectorXd> polygons_b_vectors_;
		void CreatePolygonIneqForm();
		Eigen::MatrixXd ConstructAMatrixFromPolygon(
				const std::vector<Eigen::Vector2d> &points
				);
		Eigen::VectorXd ConstructBVectorFromPolygon(
				const std::vector<Eigen::Vector2d> &points,
				const Eigen::MatrixXd &A
				);
		Eigen::Vector2d GetNormalPointingInwards(
				const Eigen::Vector2d &p1,
				const Eigen::Vector2d &p2
				);

		void SolveOptimization();

		// **************** //
		// HELPER FUNCTIONS //
		// **************** //

		int GetSegmentIndexForT(const double t_abs);
		double GetTimeInSegmentNormalized(
				const double t_abs, const int traj_segment_index
				);

		symbolic_vector_t GetZmpExpressionAtT(const double t_abs);
		symbolic_vector_t GetPosExpressionAtT(const double t_abs);
		symbolic_vector_t GetAccExpressionAtT(const double t_abs);

		Eigen::MatrixXd GetTransformationMatrixAtT(
				double t, symbolic_vector_t v
				);
		symbolic_vector_t GetPosExpressionAtT(
				double t_in_segment_normalized, int segment_j
				);
		symbolic_vector_t GetVelExpressionAtT(
				double t_in_segment_normalized, int segment_j
				);
		symbolic_vector_t GetAccExpressionAtT(
				double t_in_segment_normalized, int segment_j
				);
		symbolic_vector_t GetTrajExpressionAtT(
				double t, symbolic_vector_t v, int segment_j
				);
		Eigen::Vector2d GetPolygonCentroid(
				const std::vector<Eigen::Vector2d> &polygon
				);
		int GetGaitStepFromTime(const double t_abs);
};

