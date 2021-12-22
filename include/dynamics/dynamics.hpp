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

		void GetFeetPositions();
		void Test();

	private:
		std::string urdf_filename_;
		pinocchio::Model model_;
		pinocchio::Data data_;
};
