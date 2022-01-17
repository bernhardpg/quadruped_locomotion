#pragma once


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
#include "helper_functions.hpp"

// TODO: Move this into its own leg motion planner
struct LegMotion
{
	double t_liftoff;
	double t_touchdown;
	Eigen::Vector2d start_pos;
	Eigen::Vector2d end_pos;
};

struct LegTrajectory 
{
	double start_time;
	double end_time;
	drake::trajectories::PiecewisePolynomial<double> xy;
	drake::trajectories::PiecewisePolynomial<double> z;
	drake::trajectories::PiecewisePolynomial<double> d_xy;
	drake::trajectories::PiecewisePolynomial<double> d_z;
	drake::trajectories::PiecewisePolynomial<double> dd_xy;
	drake::trajectories::PiecewisePolynomial<double> dd_z;
};

std::ostream& operator<<(std::ostream& os, const LegMotion& lm) {
    return os << "t_liftoff: " << lm.t_liftoff << std::endl
              << "t_touchdown: " << lm.t_touchdown << std::endl
              << "start_pos: " << lm.start_pos.transpose() << std::endl
              << "end_pos: " << lm.end_pos.transpose() << std::endl;

}

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
		void PublishLegTrajectoriesVisualization();

		// LEGS // 
		// TODO: Move into own module

		Eigen::VectorXd EvalLegPosAtT(double t, int leg_i);
		Eigen::VectorXd EvalLegVelAtT(double t, int leg_i);
		Eigen::VectorXd EvalLegAccAtT(double t, int leg_i);

		Eigen::VectorXd GetLegsInContactAtT(double time);
		Eigen::MatrixXd GetStackedLegPosAtT(double time);
		Eigen::MatrixXd GetStackedLegVelAtT(double time);
		Eigen::MatrixXd GetStackedLegAccAtT(double time);

		void PublishLegTrajectories(double time);
		void PublishLegsInContact(double time);
		void PublishLegPosCmd(double time);
		void PublishLegVelCmd(double time);
		void PublishLegAccCmd(double time);

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

		void InitGaitSequence();
		void GenerateSupportPolygons();

		// TODO: move these into a feet motion planner
		std::vector<LegMotion> leg_motions_;
		std::vector<LegTrajectory> leg_trajectories_;
		std::vector<LegMotion> CreateLegMotions(); 
		std::vector<LegTrajectory> CreateLegTrajectories(
				std::vector<LegMotion> leg_motions
				);
		LegMotion CreateLegMotionForLeg(const int leg_i);
		drake::trajectories::PiecewisePolynomial<double>
			CreateXYLegTrajectory(LegMotion leg_motion);
		drake::trajectories::PiecewisePolynomial<double>
			CreateZLegTrajectory(LegMotion leg_motion);

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
		int GetLegStateAtStep(int gait_step_i, int leg_i);
		double GetTimeAtGaitStep(int gait_step_i);
		Eigen::VectorXd GetFootPosAtStep(int gait_step_i, int leg_i);

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

