#pragma once
#include <stdio.h>
#include <Eigen/Core>
#include "variable_types.hpp"
#include "control/ho_qp/task_definition.hpp"

void PrintMatrix(Eigen::MatrixXd matr)
{
	std::cout << std::setprecision(2) << std::fixed
		<< matr << std::endl << std::endl;	
}

void PrintMatrixSize(
		std::string name, Eigen::MatrixXd matr
		)
{
	std::cout << name << ": " << matr.rows() << "x" << matr.cols()
		<< std::endl;
}

void PrintMatrixSize(
		std::string name, Eigen::VectorXd matr
		)
{
	std::cout << name << ": " << matr.rows() << "x" << matr.cols()
		<< std::endl;
}

void PrintMatrixSize(
		std::string name, symbolic_vector_t matr
		)
{
	std::cout << name << ": " << matr.rows() << "x" << matr.cols()
		<< std::endl;
}

void CheckSolutionValid(TaskDefinition task, Eigen::VectorXd sol)
{
	std::cout << "=== Checking solution ===\n";
	double eps = 1e-4;
	if (task.A.rows() > 0)
	{
		Eigen::VectorXd residue = task.A*sol - task.b;
		std::cout << "Ax-b = \n";
		PrintMatrix(residue);
		if ((residue.array().abs() > eps).any())
		{
			std::cout << "Solution not valid: Equality constraint violated" << std::endl;
			return;
		}
	}
	if (task.D.rows() > 0)
	{
		Eigen::VectorXd residue = task.D*sol - task.f;
		std::cout << "Dx-f = " << std::endl;
		PrintMatrix(residue);
		if ((residue.array() > 0).any())
		{
			std::cout << "Solution not valid: Inequality constraint violated" << std::endl;
			return;
		}
	}
	std::cout << "Solution valid.\n";
	std::cout << "===============\n";
}

void CheckSolutionValid(
		Eigen::MatrixXd A, Eigen::VectorXd b,
		Eigen::MatrixXd D, Eigen::VectorXd f,
		Eigen::VectorXd sol)
{
	std::cout << "=== Checking solution ===\n";
	double eps = 1e-4;
	if (A.rows() > 0)
	{
		Eigen::VectorXd residue = A*sol - b;
		std::cout << "Ax-b = \n";
		PrintMatrix(residue);
		if ((residue.array().abs() > eps).any())
		{
			std::cout << "Solution not valid: Equality constraint violated" << std::endl;
			return;
		}
	}
	if (D.rows() > 0)
	{
		Eigen::VectorXd residue = D*sol - f;
		std::cout << "Dx-f = " << std::endl;
		PrintMatrix(residue);
		if ((residue.array() > 0).any())
		{
			std::cout << "Solution not valid: Inequality constraint violated" << std::endl;
			return;
		}
	}
	std::cout << "Solution valid.\n";
	std::cout << "===============\n";
}

void PrintTask(TaskDefinition task)
{
	std::cout << "A:" << std::endl;
	PrintMatrix(task.A);
	std::cout << "b:" << std::endl;
	PrintMatrix(task.b);
	std::cout << "D:" << std::endl;
	PrintMatrix(task.D);
	std::cout << "f:" << std::endl;
	PrintMatrix(task.f);
}

