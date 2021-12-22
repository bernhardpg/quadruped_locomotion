#pragma once

#include <pinocchio/fwd.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <iostream>
#include <iomanip>

class Dynamics
{
	public:
		Dynamics();

		Eigen::Matrix<double,12,1> GetFeetPositions(
				Eigen::Matrix<double, 19, 1> q
				);
		Eigen::MatrixXd GetFootJacobian(
				Eigen::Matrix<double,19,1> q, int foot_i
				);
		Eigen::MatrixXd GetStackedFeetJacobian(
				Eigen::Matrix<double,19,1> q
				);
		Eigen::MatrixXd GetStackedFeetJacobianPos(
				Eigen::Matrix<double,19,1> q
				);
		void Test(); // TODO: Remove

	private:
		int n_legs_ = 4;
		int n_dims_ = 3;

		std::string urdf_filename_;
		std::vector<std::string> feet_frames_;
		pinocchio::Model model_;
		pinocchio::Data data_;
};
