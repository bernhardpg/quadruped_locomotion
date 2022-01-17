#pragma once


#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#include <drake/solvers/mathematical_program.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>
#include <Eigen/Core>
#include <iomanip>

#include "variable_types.hpp"
#include "dynamics/dynamics.hpp"
#include "helper_functions.hpp"

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
		void PublishPolygonVisualizationAtTime(double time);
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

		Dynamics robot_dynamics_;

		int n_legs_ = 4;
		int n_gait_steps_;
		int n_gait_dims_;;
		double t_per_gait_sequence_;
		double t_per_step_;
		Eigen::MatrixXd gait_sequence_;

		Eigen::Vector2d vel_cmd_;
		int curr_gait_step_;
		Eigen::MatrixXd current_stance_;
		std::vector<Eigen::MatrixXd> stance_sequence_;

		int degree_;
		int n_traj_segments_;
		int traj_dimension_ = 2;
		double dt_ = 0.1;
		Eigen::Vector2d pos_initial_;
		Eigen::Vector2d pos_final_;

		std::vector<std::vector<Eigen::Vector2d>> support_polygons_;

		void AddTestPolygons();
		void InitGaitSequence();
		void GenerateSupportPolygons();

		std::vector<Eigen::MatrixXd> GenerateStanceSequence(
				const Eigen::VectorXd &vel_cmd,
				const Eigen::MatrixXd &current_stance
				);
		Eigen::MatrixXd GenerateStanceForNextTimestep(
				const Eigen::VectorXd &vel_cmd,
				const int gait_step_i,
				const Eigen::MatrixXd &prev_stance
				);
		int GetGaitStepFromTime(double t);

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

