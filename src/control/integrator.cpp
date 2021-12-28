#include "control/integrator.hpp"

Integrator::Integrator(int rows)
{
	SetSize(rows);
	Reset();
}

void Integrator::SetSize(int rows)
{
	integral_.resize(rows);
}

void Integrator::Reset()
{
	integral_.setZero();
	ResetTime();
}

Eigen::VectorXd Integrator::GetIntegral()
{
	return integral_;
}

void Integrator::SetIntegral(Eigen::VectorXd target)
{
	integral_ = target;
	ResetTime();
}

void Integrator::Integrate(Eigen::VectorXd vec)
{
	dt_ = GetElapsedTimeSince(last_timestamp_);
	integral_ += dt_ * vec;
	ResetTime();
}

void Integrator::ResetTime()
{
	last_timestamp_ = ros::Time::now();
}

double Integrator::GetElapsedTimeSince(ros::Time t)
{
	double elapsed_time = (ros::Time::now() - t).toSec();
	return elapsed_time; 
}
