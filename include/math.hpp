#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues> 
#include <assert.h>
#include <stdio.h>

const double kInf = 9999999;
const double kEps = 1e-7;

bool CheckEigenValuesPositive(Eigen::MatrixXd m)
{
	Eigen::EigenSolver<Eigen::MatrixXd> es(m, false);
	auto eigenvalues = es.eigenvalues();

	for (int i = 0; i < eigenvalues.rows(); ++i)
		if (eigenvalues(i).real() < 0.0) return false;

	return true;
}

Eigen::MatrixXd ConcatenateMatrices(
		Eigen::MatrixXd m1, Eigen::MatrixXd m2
		)
{
	if (m1.cols() == 0)
		return m2;
	else if (m2.cols() == 0)
		return m1;

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
	if (v1.cols() == 0)
		return v2;
	else if (v2.cols() == 0)
		return v1;

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

Eigen::MatrixXd CalcPseudoInverse(
		Eigen::MatrixXd A, double damping
		)
{
	// Same size as A*A^t
	Eigen::MatrixXd eye =
		Eigen::MatrixXd::Identity(A.rows(), A.rows());

	// Moore-Penrose right inverse with damping: A^t (A A^t + lambda I)
	Eigen:: MatrixXd pseudo_inverse =
		A.transpose() * (A * A.transpose() + damping * eye).inverse();
	return pseudo_inverse;
}

Eigen::MatrixXd CalcPseudoInverse(Eigen::MatrixXd A)
{
	// Moore-Penrose right inverse: A^t (A A^t)
	return CalcPseudoInverse(A, kEps); 
}

Eigen::MatrixXd CalcNullSpaceProjMatrix(Eigen::MatrixXd A)
{
	assert((A.rows() > 0) && (A.cols() > 0));

	// Alternative computation approaches can be found here:
	// https://eigen.tuxfamily.org/dox/group__LeastSquares.html
	Eigen::MatrixXd A_inv = CalcPseudoInverse(A);
	Eigen::MatrixXd eye =
		Eigen::MatrixXd::Identity(A.cols(), A.cols());

	Eigen::MatrixXd null_space_projection_matrix = eye - A_inv * A;
	return null_space_projection_matrix;
}

