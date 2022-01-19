#pragma once

#include <Eigen/Core>
#include <drake/common/trajectories/piecewise_polynomial.h>

#include "anymal_constants.hpp"
#include "planner/gait_sequence.hpp"

struct LegMotion;
struct LegTrajectory;

class LegPlanner
{
	public:
		LegPlanner(){};

		void PlanLegsMotion(
				const Eigen::VectorXd &vel_cmd,
				const Eigen::MatrixXd &current_stance
				);

		// ***************** //
		// SETTERS & GETTERS //
		// ***************** //

		void SetGaitSequence(
				const GaitSequence &gait_sequence
				);

		std::vector<Eigen::Vector2d> GetSupportPolygonAtT(const double time);
		std::vector<std::vector<Eigen::Vector2d>> GetSupportPolygons();

		double GetLegTrajStartTime(const int leg_i);
		double GetLegTrajEndTime(const int leg_i);
		Eigen::VectorXd GetLegPosAtT(const double time, const int leg_i);

		Eigen::VectorXd GetAllLegsContact();
		Eigen::VectorXd GetLegsInContactAtT(const double time);
		Eigen::VectorXd GetStackedLegPosAtT(const double time);
		Eigen::VectorXd GetStackedLegVelAtT(const double time);
		Eigen::VectorXd GetStackedLegAccAtT(const double time);

	private:
		GaitSequence gait_sequence_;
		std::vector<Eigen::MatrixXd> stance_sequence_;
		std::vector<std::vector<Eigen::Vector2d>> support_polygons_;

		std::vector<LegMotion> leg_motions_;
		std::vector<LegTrajectory> leg_trajectories_;

		// ***************** //
		// STANCE GENERATION //
		// ***************** //

		std::vector<Eigen::MatrixXd> GenerateStanceSequence(
				const Eigen::VectorXd &vel_cmd,
				const Eigen::MatrixXd &current_stance
				);
		Eigen::MatrixXd GenerateStanceForNextTimestep(
				const Eigen::VectorXd &vel_cmd,
				const Eigen::MatrixXd &prev_stance,
				const int gait_step_i
				);
		std::vector<std::vector<Eigen::Vector2d>>
			GenerateSupportPolygons();

		// ********************** //
		// SWING LEG TRAJECTORIES //
		// ********************** //

		std::vector<LegTrajectory> CreateLegTrajectories(
				const std::vector<LegMotion> &leg_motions
				);
		std::vector<LegMotion> CreateLegMotions(); 
		LegMotion CreateLegMotionForLeg(const int leg_i);

		drake::trajectories::PiecewisePolynomial<double>
			CreateXYLegTrajectory(const LegMotion &leg_motion);
		drake::trajectories::PiecewisePolynomial<double>
			CreateZLegTrajectory(const LegMotion &leg_motion);

		Eigen::VectorXd EvalLegPosAtT(const double t, const int leg_i);
		Eigen::VectorXd EvalLegVelAtT(const double t, const int leg_i);
		Eigen::VectorXd EvalLegAccAtT(const double t, const int leg_i);

		// **************** //
		// HELPER FUNCTIONS //
		// **************** //

		int GetLegStateAtStep(int gait_step_i, int leg_i);
		double GetTimeAtGaitStep(int gait_step_i);
		Eigen::VectorXd GetFootPosAtStep(int gait_step_i, int leg_i);
		int GetGaitStepFromTime(double t);
};

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

