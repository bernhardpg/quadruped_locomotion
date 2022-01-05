#pragma once
#include <stdio.h>
#include <Eigen/Core>
#include "variable_types.hpp"

void PrintMatrix(Eigen::MatrixXd matr)
{
	std::cout << std::setprecision(3) << std::fixed
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
		if (residue.sum() > eps)
		{
			std::cout << "Solution not valid: Equality constraint violated" << std::endl;
			std::cout << "Ax-b = " << std::endl;
			PrintMatrix(residue);
			return;
		}
	}
	if (D.rows() > 0)
	{
		Eigen::VectorXd residue = D*sol - f;
		if ((residue.array() > 0).any())
		{
			std::cout << "Solution not valid: Inequality constraint violated" << std::endl;
			std::cout << "Dx-f = " << std::endl;
			PrintMatrix(residue);
			return;
		}
	}
	std::cout << "Solution valid.\n";
	std::cout << "===============\n";
}
