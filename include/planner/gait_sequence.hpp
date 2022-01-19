#pragma once

struct GaitSequence
{
	int n_steps;
	double duration;
	double step_time;
	Eigen::MatrixXd contact_schedule;
};

