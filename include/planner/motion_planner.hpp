#pragma once

#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#include <drake/solvers/mathematical_program.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/symbolic.h>
#include <drake/solvers/solve.h>
#include <Eigen/Core>
#include <iomanip>

#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>

#include "variable_types.hpp"
#include "dynamics/dynamics.hpp"
#include "planner/leg_planner.hpp"
#include "planner/gait_sequence.hpp"
#include "helper_functions.hpp"
#include "anymal_constants.hpp"

class MotionPlanner
{
	public:
		MotionPlanner(int degree, int num_traj_segments);

		void GenerateTrajectory();

		// ************* //
		// VISUALIZATION //
		// ************* //

		void PublishComTrajVisualization();
		void PublishPolygonVisualizationAtTime(const double time);
		void PublishLegTrajectoriesVisualization();

		// COM //
		// TODO: move into its own module
		Eigen::VectorXd EvalComTrajAtT(
				const double t, const int derivative = 0
				);
		Eigen::VectorXd EvalComPosAtT(const double t);
		Eigen::VectorXd EvalComVelAtT(const double t);
		Eigen::VectorXd EvalComAccAtT(const double t);

		void PublishComTrajectories(const double time);
		void PublishComPosCmd(const double time);
		void PublishComVelCmd(const double time);
		void PublishComAccCmd(const double time);

		void PublishLegTrajectories(const double time);
		void PublishLegsInContact(const double time);
		void PublishLegPosCmd(const double time);
		void PublishLegVelCmd(const double time);
		void PublishLegAccCmd(const double time);

	private:
		// *** //
		// ROS //
		// *** //

		ros::NodeHandle ros_node_;
		ros::Publisher visualize_com_traj_pub_; 
		ros::Publisher visualize_leg_traj_pub_; 
		ros::Publisher visualize_polygons_pub_; 

		ros::Publisher legs_in_contact_pub_; 
		ros::Publisher legs_pos_cmd_pub_; 
		ros::Publisher legs_vel_cmd_pub_; 
		ros::Publisher legs_acc_cmd_pub_; 

		ros::Publisher com_pos_traj_pub_; 
		ros::Publisher com_vel_traj_pub_; 
		ros::Publisher com_acc_traj_pub_; 

		void InitRos();
		void SetupVisualizationTopics();
		void SetupComCmdTopics();
		void SetupLegCmdTopics();

		// ******************* //
		// GAIT AND TRAJECTORY //
		// ******************* //
		LegPlanner leg_planner_;

		Dynamics robot_dynamics_;

		GaitSequence gait_sequence_;

		Eigen::Vector2d vel_cmd_;
		int curr_gait_step_;
		Eigen::MatrixXd current_stance_;
		std::vector<Eigen::MatrixXd> stance_sequence_;

		int degree_;
		int n_traj_segments_;
		const int traj_dimension_ = k2D;
		double dt_ = 0.1;
		Eigen::Vector2d pos_initial_;
		Eigen::Vector2d pos_final_;

		GaitSequence CreateCrawlSequence();

		std::vector<Eigen::MatrixXd> GenerateStanceSequence(
				const Eigen::VectorXd &vel_cmd,
				const Eigen::MatrixXd &current_stance
				);
		Eigen::MatrixXd GenerateStanceForNextTimestep(
				const Eigen::VectorXd &vel_cmd,
				const int gait_step_i,
				const Eigen::MatrixXd &prev_stance
				);

		// ******************** //
		// OPTIMIZATION PROBLEM //
		// ******************** //

		void SetupOptimizationProgram();
		void InitMonomials();
		void InitDecisionVariables();
		void AddAccelerationCost();
		void AddContinuityConstraints();
		void AddInitialAndFinalConstraint();

		void GeneratePolynomialsFromSolution();
		std::vector<symbolic_matrix_t> GetCoeffValues();
		polynomial_matrix_t GeneratePolynomials(
				const symbolic_vector_t &monomial_basis,
				const std::vector<symbolic_matrix_t> &coeff_values 
				);

		drake::solvers::MathematicalProgram prog_;
		drake::solvers::MathematicalProgramResult result_;
		polynomial_matrix_t polynomials_pos_;
		polynomial_matrix_t polynomials_vel_;
		polynomial_matrix_t polynomials_acc_;

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
};

