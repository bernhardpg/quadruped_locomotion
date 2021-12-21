#pragma once


#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>
#include <Eigen/Core>
#include <iomanip>

// TODO: Move these into their own file? They are not specific to MotionPlanner
typedef Eigen::Matrix<drake::symbolic::Expression, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> symbolic_matrix_t; // Specify to store data in RowOrder to make it easy to transform this into a vector
typedef Eigen::Matrix<drake::symbolic::Expression, Eigen::Dynamic, 1> symbolic_vector_t;
typedef Eigen::Matrix<drake::symbolic::Polynomial, Eigen::Dynamic, Eigen::Dynamic> polynomial_matrix_t;

class MotionPlanner
{
	public:
		MotionPlanner(int degree, int num_traj_segments);

		void GenerateTrajectory();
		Eigen::VectorXd EvalTrajAtT(double t);

		// ************* //
		// VISUALIZATION //
		// ************* //
		void PublishTrajectoryVisualization();
		void PublishPolygonsVisualization();

	private:
		// *** //
		// ROS //
		// *** //
		ros::NodeHandle ros_node_;
		ros::Publisher traj_pub_;
		ros::Publisher polygons_pub_;
		void InitRos();

		// ******************* //
		// GAIT AND TRAJECTORY //
		// ******************* //
		Eigen::MatrixXd gait_sequence_;
		std::vector<std::vector<Eigen::Vector2d>> support_polygons_;
		int degree_;
		int n_traj_segments_;
		int traj_dimension_ = 2;
		double dt_ = 0.1;
		Eigen::Vector2d pos_initial_;
		Eigen::Vector2d pos_final_;

		void AddTestPolygons();
		void InitGaitSequence();

		// ******************** //
		// OPTIMIZATION PROBLEM //
		// ******************** //
		void SetupOptimizationProgram();
		void InitDecisionVariables();
		void AddAccelerationCost();
		void AddContinuityConstraints();
		void AddInitialAndFinalConstraint();
		void GeneratePolynomialsFromSolution();

		drake::solvers::MathematicalProgram prog_;
		drake::solvers::MathematicalProgramResult result_;
		polynomial_matrix_t polynomials_;

		drake::symbolic::Variable t_;
		symbolic_vector_t m_; // monomial basis of t
		symbolic_vector_t m_dot_;
		symbolic_vector_t m_ddot_;
		symbolic_matrix_t T_ ; // Transformation matrices for m
		symbolic_matrix_t T_dot_;
		symbolic_matrix_t T_ddot_;

		std::vector<symbolic_matrix_t> coeffs_;

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
				std::vector<Eigen::Vector2d> polygon
				);

};

