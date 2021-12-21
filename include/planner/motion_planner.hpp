#pragma once


#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>
#include <Eigen/Core>
#include <iomanip>

typedef Eigen::Matrix<drake::symbolic::Expression, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> symbolic_matrix_t; // Specify to store data in RowOrder to make it easy to transform this into a vector
typedef Eigen::Matrix<drake::symbolic::Expression, Eigen::Dynamic, 1> symbolic_vector_t;
typedef Eigen::Matrix<drake::symbolic::Polynomial, Eigen::Dynamic, Eigen::Dynamic> polynomial_matrix_t;

class MotionPlanner
{
	public:
		MotionPlanner(int degree, int num_traj_segments);

		void GenerateTrajectory();
		Eigen::VectorXd EvalTrajAtT(double t);
		void PublishTrajectory();
		void PublishPolygons();

	private:
		// ROS
		ros::NodeHandle ros_node_;
		ros::Publisher traj_pub_;
		ros::Publisher polygons_pub_;
		void InitRos();

		void SetupOptimizationProgram();
		void InitDecisionVariables();
		void AddAccelerationCost();
		void AddContinuityConstraints();
		void AddInitialAndFinalConstraint();
		void AddTestPolygons();

		int degree_;
		int n_traj_segments_;
		int traj_dimension_ = 2;
		double dt_ = 0.1;
		std::vector<std::vector<Eigen::Vector2d>> support_polygons_;

		drake::solvers::MathematicalProgram prog_;
		drake::solvers::MathematicalProgramResult result_;
		polynomial_matrix_t polynomials_;

		// TODO: Hardcoded for now
		Eigen::Vector2d pos_initial_;
		Eigen::Vector2d pos_final_;

		drake::symbolic::Variable t_;

		// Monomial basis of t
		symbolic_vector_t m_;
		symbolic_vector_t m_dot_;
		symbolic_vector_t m_ddot_;

		// Transformation matrices for m
		symbolic_matrix_t T_ ;
		symbolic_matrix_t T_dot_;
		symbolic_matrix_t T_ddot_;

		std::vector<symbolic_matrix_t> coeffs_;

		void construct_transform_matrix(
				symbolic_matrix_t *T, symbolic_vector_t *v
				);

		Eigen::MatrixXd GetTransformationMatrixAtT(
				double t, symbolic_vector_t v
				);


		symbolic_vector_t GetPosAtT(double t);
		symbolic_vector_t GetPosAtT(double t_in_segment, int segment_j);
		symbolic_vector_t GetVelAtT(double t_in_segment, int segment_j);
		symbolic_vector_t GetAccAtT(double t_in_segment, int segment_j);

		symbolic_vector_t GetTrajExpressionAtT(
				double t, symbolic_vector_t v, int segment_j
				);

		void GeneratePolynomials();
};
