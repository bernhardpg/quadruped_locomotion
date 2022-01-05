#pragma once
#include <Eigen/Core>

struct TaskDefinition
{
	Eigen::MatrixXd A;
	Eigen::VectorXd b;
	Eigen::MatrixXd D;
	Eigen::VectorXd f;
};
