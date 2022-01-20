#pragma once
#include <Eigen/Core>
#include "math.hpp"

struct TaskDefinition
{
	Eigen::MatrixXd A;
	Eigen::VectorXd b;
	Eigen::MatrixXd D;
	Eigen::VectorXd f;
};

TaskDefinition ConcatenateTasks(
		const TaskDefinition &t1, const TaskDefinition &t2
		)
{
	Eigen::MatrixXd A_concat = 
		ConcatenateMatrices(t1.A, t2.A);
	Eigen::VectorXd b_concat =
		ConcatenateVectors(t1.b, t2.b);
	Eigen::MatrixXd D_concat =
		ConcatenateMatrices(t1.D, t2.D);
	Eigen::VectorXd f_concat =
		ConcatenateVectors(t1.f, t2.f);

	TaskDefinition res =
		{A_concat, b_concat, D_concat, f_concat};
	
	return res;
}

TaskDefinition ConcatenateTasks(
		const std::vector<TaskDefinition> &tasks
		)
{
	TaskDefinition res = tasks[0];
	for (int i = 1; i < tasks.size(); ++i)
		res = ConcatenateTasks(res,tasks[i]);
	
	return res;
}

