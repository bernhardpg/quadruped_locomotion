#pragma once
#include <drake/common/symbolic.h>

typedef Eigen::Matrix<drake::symbolic::Expression, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> symbolic_matrix_t; // Specify to store data in RowOrder to make it easy to transform this into a vector
typedef Eigen::Matrix<drake::symbolic::Expression, Eigen::Dynamic, 1> symbolic_vector_t;
typedef Eigen::Matrix<drake::symbolic::Polynomial, Eigen::Dynamic, Eigen::Dynamic> polynomial_matrix_t;
