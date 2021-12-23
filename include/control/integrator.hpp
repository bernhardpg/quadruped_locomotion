#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Core>

//TODO: Move this to another folder when used by anything else than controller

class Integrator
{
	public:
		Integrator() : last_timestamp_(0){};
		Integrator(int rows);

		void Integrate(Eigen::VectorXd vec);
		void SetSize(int rows);
		void Reset();

		Eigen::VectorXd GetIntegral();
		void SetIntegral(Eigen::VectorXd target);

	private:
		double dt_;
		ros::Time last_timestamp_;
		Eigen::VectorXd integral_;

		double GetElapsedTimeSince(ros::Time t);
};
