#pragma once

#include <Eigen/Core>

const double kInf = 9999999;

Eigen::MatrixXd ConcatenateMatrices(
		Eigen::MatrixXd m1, Eigen::MatrixXd m2
		)
{
	assert (m1.cols() == m2.cols());
	Eigen::MatrixXd res(m1.rows() + m2.rows(), m1.cols());
	res << m1,
				 m2;

	return res;
}

Eigen::VectorXd ConcatenateVectors(
		Eigen::VectorXd v1, Eigen::VectorXd v2
		)
{
	assert (v1.cols() == v2.cols());
	Eigen::VectorXd res(v1.rows() + v2.rows());
	res << v1,
				 v2;

	return res;
}

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

