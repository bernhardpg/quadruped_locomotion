#pragma once

#include <pinocchio/fwd.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <iostream>
#include <iomanip>

#include "anymal_constants.hpp"

class Dynamics
{
	public:
		Dynamics();

		Eigen::Matrix<double,kNumFeetCoords,1> GetFeetPositions(
				Eigen::Matrix<double,kNumGenCoords, 1> q
				);
		Eigen::MatrixXd GetContactJacobian(
				Eigen::Matrix<double,kNumGenCoords,1> q, int foot_i
				);
		Eigen::MatrixXd GetStackedContactJacobianPos(
				Eigen::Matrix<double,kNumGenCoords,1> q
				);
		void Test(); // TODO: Remove

	private:
		std::string urdf_filename_;
		pinocchio::Model model_;
		pinocchio::Data data_;
};
