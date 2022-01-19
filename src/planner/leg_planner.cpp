#include "planner/leg_planner.hpp"

void LegPlanner::PlanLegsMotion(
		const Eigen::VectorXd &vel_cmd,
		const Eigen::MatrixXd &current_stance
		)
{
	stance_sequence_ = GenerateStanceSequence(vel_cmd, current_stance);
	leg_motions_ = CreateLegMotions();
	leg_trajectories_ = CreateLegTrajectories(leg_motions_);
	support_polygons_ = GenerateSupportPolygons();
}

// ***************** //
// SETTERS & GETTERS //
// ***************** //

void LegPlanner::SetGaitSequence(
		const GaitSequence &gait_sequence
		)
{
	gait_sequence_ = gait_sequence;
}

std::vector<Eigen::Vector2d> LegPlanner::GetSupportPolygonAtT(
		const double time
		)
{
	const int polygon_i = GetGaitStepFromTime(time);
	return support_polygons_[polygon_i];
}

std::vector<std::vector<Eigen::Vector2d>>
	LegPlanner::GetSupportPolygons()
{
	return support_polygons_;
}

double LegPlanner::GetLegTrajStartTime(const int leg_i)
{
	return leg_trajectories_[leg_i].start_time;
}

double LegPlanner::GetLegTrajEndTime(const int leg_i)
{
	return leg_trajectories_[leg_i].end_time;
}

Eigen::VectorXd LegPlanner::GetLegPosAtT(
		const double time, const int leg_i
		)
{
	return EvalLegPosAtT(time, leg_i);
}

Eigen::VectorXd LegPlanner::GetLegsInContactAtT(const double time)
{
	int i = GetGaitStepFromTime(time);
	return gait_sequence_.contact_schedule.col(i);
}

Eigen::VectorXd LegPlanner::GetStackedLegPosAtT(const double time)
{
	Eigen::VectorXd stacked_leg_pos(k3D * kNumLegs);
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		stacked_leg_pos.block<k3D,1>(leg_i * k3D,0) =
			EvalLegPosAtT(time, leg_i);
	}
	return stacked_leg_pos;
}

Eigen::VectorXd LegPlanner::GetStackedLegVelAtT(const double time)
{
	Eigen::VectorXd stacked_leg_vel(k3D * kNumLegs);
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		stacked_leg_vel.block<k3D,1>(leg_i * k3D,0) =
			EvalLegVelAtT(time, leg_i);
	}
	return stacked_leg_vel;
}

Eigen::VectorXd LegPlanner::GetStackedLegAccAtT(const double time)
{
	Eigen::VectorXd stacked_leg_acc(k3D * kNumLegs);
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		stacked_leg_acc.block<k3D,1>(leg_i * k3D,0) =
			EvalLegAccAtT(time, leg_i);
	}
	return stacked_leg_acc;
}

// ***************** //
// STANCE GENERATION //
// ***************** //

std::vector<Eigen::MatrixXd> LegPlanner::GenerateStanceSequence(
		const Eigen::VectorXd &vel_cmd,
		const Eigen::MatrixXd &current_stance
		)
{
	std::vector<Eigen::MatrixXd> stance_sequence;
	stance_sequence.push_back(current_stance);
	for (int gait_step_i = 1; gait_step_i < gait_sequence_.n_steps; ++gait_step_i)
	{
		Eigen::MatrixXd next_stance =
			GenerateStanceForNextTimestep(
					vel_cmd, stance_sequence[gait_step_i - 1], gait_step_i
					);
		stance_sequence.push_back(next_stance);
	}
	return stance_sequence;
}

Eigen::MatrixXd LegPlanner::GenerateStanceForNextTimestep(
		const Eigen::VectorXd &vel_cmd,
		const Eigen::MatrixXd &prev_stance,
		const int gait_step_i
		)
{
	Eigen::MatrixXd next_stance(k2D, kNumLegs);
	next_stance.setZero();

	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		Eigen::MatrixXd prev_foot_pos = prev_stance.col(leg_i);

		Eigen::MatrixXd next_foot_pos(k2D,1);
		if (GetLegStateAtStep(gait_step_i, leg_i) == 1)
		{
			next_foot_pos = prev_foot_pos;
		}
		else
		{
			next_foot_pos = prev_foot_pos
				+ vel_cmd * gait_sequence_.step_time;
		}
		
		next_stance.col(leg_i) = next_foot_pos;
	}

	return next_stance;
}

std::vector<std::vector<Eigen::Vector2d>>
	LegPlanner::GenerateSupportPolygons()
{
	std::vector<std::vector<Eigen::Vector2d>> support_polygons;

	std::vector<Eigen::Vector2d> new_polygon;
	for (int gait_step_i = 0; gait_step_i < gait_sequence_.n_steps; ++gait_step_i)
	{
		new_polygon.clear();
		auto curr_stance = stance_sequence_[gait_step_i];

		std::vector<int> leg_visualization_order = {0,2,3,1}; 
		for (int leg_i : leg_visualization_order)	
		{
			if (GetLegStateAtStep(gait_step_i, leg_i) == 1)	
			{
				new_polygon.push_back(curr_stance.col(leg_i));
			}
		}
		support_polygons.push_back(new_polygon);
	}

	return support_polygons;
}

// ********************** //
// SWING LEG TRAJECTORIES //
// ********************** //

std::vector<LegTrajectory> LegPlanner::CreateLegTrajectories(
		const std::vector<LegMotion> &leg_motions
		)
{
	std::vector<LegTrajectory> leg_trajs;
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		LegTrajectory leg_i_traj; 

		leg_i_traj.start_time = leg_motions[leg_i].t_liftoff;
		leg_i_traj.end_time = leg_motions[leg_i].t_touchdown;

		leg_i_traj.xy = CreateXYLegTrajectory(leg_motions[leg_i]);
		leg_i_traj.z = CreateZLegTrajectory(leg_motions[leg_i]);

		leg_i_traj.d_xy = leg_i_traj.xy.derivative(1);
		leg_i_traj.d_z = leg_i_traj.z.derivative(1);

		leg_i_traj.dd_xy = leg_i_traj.xy.derivative(2);
		leg_i_traj.dd_z = leg_i_traj.z.derivative(2);

		leg_trajs.push_back(leg_i_traj);
	}

	return leg_trajs;
}

drake::trajectories::PiecewisePolynomial<double>
	LegPlanner::CreateXYLegTrajectory(const LegMotion &leg_motion)
{
	const std::vector<double> breaks = {
		leg_motion.t_liftoff, leg_motion.t_touchdown
	};

	std::vector<Eigen::MatrixXd> samples = {
		leg_motion.start_pos, leg_motion.end_pos
	};

	drake::trajectories::PiecewisePolynomial<double> xy_traj =
	 	drake::trajectories::PiecewisePolynomial<double>
				::FirstOrderHold(breaks, samples);

	return xy_traj;
}

drake::trajectories::PiecewisePolynomial<double>
	LegPlanner::CreateZLegTrajectory(const LegMotion &leg_motion)
{
	const double t_apex = leg_motion.t_liftoff + std::abs(
			leg_motion.t_touchdown - leg_motion.t_liftoff
			) / 2;
	const std::vector<double> breaks = {
		leg_motion.t_liftoff, t_apex, leg_motion.t_touchdown
	};

	const double z_apex_height = 0.2; // TODO: Move to member variable
	Eigen::MatrixXd apex(1,1);
	apex << z_apex_height;

	const Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(1,1);
	const std::vector<Eigen::MatrixXd> samples = {
		zero, apex, zero
	};

	drake::trajectories::PiecewisePolynomial<double> z_traj =
	 	drake::trajectories::PiecewisePolynomial<double>
				::CubicWithContinuousSecondDerivatives(breaks, samples);

	return z_traj;
}

std::vector<LegMotion> LegPlanner::CreateLegMotions()
{
	std::vector<LegMotion> leg_motions;
	for (int leg_i = 0; leg_i < kNumLegs; ++leg_i)
	{
		leg_motions.push_back(CreateLegMotionForLeg(leg_i));
	}

	return leg_motions;
}

LegMotion LegPlanner::CreateLegMotionForLeg(const int leg_i)
{
	double t_liftoff = -1;
	double t_touchdown = -1;
	Eigen::VectorXd start_pos(k2D);
	Eigen::VectorXd end_pos(k2D);

	// TODO: generalize to start at any configuration
	int last_leg_state = GetLegStateAtStep(0, leg_i);
	for (int gait_step_i = 1;
			gait_step_i < gait_sequence_.n_steps;
			++gait_step_i)
	{
		if (GetLegStateAtStep(gait_step_i, leg_i) != last_leg_state)
		{
			if (last_leg_state == 1)	
			{
				t_liftoff = GetTimeAtGaitStep(gait_step_i);
				last_leg_state = 0;
				start_pos = GetFootPosAtStep(gait_step_i - 1, leg_i);
			}
			else
			{
				t_touchdown = GetTimeAtGaitStep(gait_step_i);
				last_leg_state = 1;
				end_pos = GetFootPosAtStep(gait_step_i - 1, leg_i);
			}
		}
	}

	LegMotion leg_i_traj = {t_liftoff, t_touchdown, start_pos, end_pos};
	return leg_i_traj;
}

Eigen::VectorXd LegPlanner::EvalLegPosAtT(
		const double time, const int leg_i
		)
{
	const double time_rel = std::fmod(time, gait_sequence_.duration);
	Eigen::VectorXd leg_pos(3);
	leg_pos.setZero();

	const bool inside_traj_time =
		(time_rel < leg_trajectories_[leg_i].start_time)
			|| (time_rel > leg_trajectories_[leg_i].end_time);

	if (!inside_traj_time)
	{
		leg_pos <<
			leg_trajectories_[leg_i].xy.value(time_rel),
			leg_trajectories_[leg_i].z.value(time_rel);
	}

	return leg_pos;
}

Eigen::VectorXd LegPlanner::EvalLegVelAtT(
		const double time, const int leg_i
		)
{
	const double time_rel = std::fmod(time, gait_sequence_.duration);
	Eigen::VectorXd leg_vel(3);
	leg_vel.setZero();

	const bool inside_traj_time =
		(time_rel < leg_trajectories_[leg_i].start_time)
			|| (time_rel > leg_trajectories_[leg_i].end_time);

	if (!inside_traj_time)
	{
		leg_vel <<
			leg_trajectories_[leg_i].xy.value(time_rel),
			leg_trajectories_[leg_i].z.value(time_rel);
	}

	return leg_vel;
}

Eigen::VectorXd LegPlanner::EvalLegAccAtT(
		const double time, const int leg_i
		)
{
	const double time_rel = std::fmod(time, gait_sequence_.duration);
	Eigen::VectorXd leg_acc(3);
	leg_acc.setZero();

	const bool inside_traj_time =
		(time_rel < leg_trajectories_[leg_i].start_time)
			|| (time_rel > leg_trajectories_[leg_i].end_time);

	if (!inside_traj_time)
	{
		leg_acc <<
			leg_trajectories_[leg_i].xy.value(time_rel),
			leg_trajectories_[leg_i].z.value(time_rel);
	}

	return leg_acc;
}

// **************** //
// HELPER FUNCTIONS //
// **************** //

double LegPlanner::GetTimeAtGaitStep(int gait_step_i)
{
	double t_at_step = gait_sequence_.step_time * gait_step_i;	
	return t_at_step;
}

Eigen::VectorXd LegPlanner::GetFootPosAtStep(
		int gait_step_i, int leg_i
		)
{
	return stance_sequence_[gait_step_i].col(leg_i);
}

int LegPlanner::GetLegStateAtStep(int gait_step_i, int leg_i)
{
	return gait_sequence_.contact_schedule(leg_i,gait_step_i);
}

int LegPlanner::GetGaitStepFromTime(double t)
{
	double t_rel = std::fmod(t, gait_sequence_.duration);
		
	int gait_step_i = t_rel / gait_sequence_.step_time;
	return gait_step_i;
}

