#pragma once
#include <stdio.h>
#include <Eigen/Core>

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
