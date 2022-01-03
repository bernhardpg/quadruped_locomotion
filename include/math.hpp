#pragma once

#include <Eigen/Core>

const double kInf = 9999999;

Eigen::VectorXd CreateInfVector(int size)
{
	Eigen::VectorXd inf_vec = Eigen::VectorXd::Zero(size);
	for (int i = 0; i < size; ++i)
		inf_vec(i) = kInf;

	return inf_vec;
}

Eigen::MatrixXd CalcPseudoInverse(Eigen::MatrixXd A)
{
	// Moore-Penrose right inverse: A^t (A A^t)
	Eigen:: MatrixXd pseudo_inverse =
		A.transpose() * (A * A.transpose()).inverse();
	return pseudo_inverse;
}

Eigen::MatrixXd CalcPseudoInverse(
		Eigen::MatrixXd A, double damping
		)
{
	// Same size as A*A^t
	Eigen::MatrixXd eye =
		Eigen::MatrixXd::Identity(A.rows(), A.rows());

	// Moore-Penrose right inverse: A^t (A A^t)
	Eigen:: MatrixXd pseudo_inverse =
		A.transpose() * (A * A.transpose() + damping * eye).inverse();
	return pseudo_inverse;
}

Eigen::MatrixXd CalcNullSpaceProjMatrix(Eigen::MatrixXd A)
{
	Eigen::MatrixXd A_inv = CalcPseudoInverse(A);
	Eigen::MatrixXd eye =
		Eigen::MatrixXd::Identity(A.cols(), A.cols());

	Eigen::MatrixXd null_space_projection_matrix = eye - A_inv * A;
	return null_space_projection_matrix;
}

