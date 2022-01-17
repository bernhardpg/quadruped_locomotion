#pragma once

#include <pinocchio/fwd.hpp>
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <iostream>
#include <iomanip>

#include "anymal_constants.hpp"
#include "helper_functions.hpp"

#include <drake/multibody/tree/multibody_element.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/geometry/scene_graph_inspector.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/common/find_resource.h>

class Dynamics
{
	public:
		Dynamics();

		// ******** //
		// DYNAMICS //
		// ******** //

		void SetState(
				Eigen::Matrix<double,kNumGenCoords, 1> q,
				Eigen::Matrix<double,kNumGenVels, 1> u
				);
		void SetStateDefault();
		Eigen::MatrixXd GetMassMatrix();
		Eigen::VectorXd GetBiasVector();
		Eigen::VectorXd GetContactAccInW(int foot_i);
		Eigen::VectorXd GetStackedContactAccInW();

		// ****************** //
		// FORWARD KINEMATICS // 
		// ****************** //

		Eigen::VectorXd GetFootPosInW(int foot_i);
		Eigen::MatrixXd GetStackedFootPosInW();
		Eigen::MatrixXd GetStacked2DFootPosInW();

		// *********************** //
		// DIFFERENTIAL KINEMATICS // 
		// *********************** //

		Eigen::MatrixXd GetContactJacobianInW(int foot_i);
		Eigen::MatrixXd GetStackedContactJacobianInW();
		Eigen::MatrixXd GetBaseJacobianInW();

	private:
		std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_;
		std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
		std::unique_ptr<drake::systems::Context<double>> context_;
		std::unique_ptr<drake::systems::Diagram<double>> diagram_;
		std::unique_ptr<drake::systems::Context<double>> diagram_context_;


		void BuildPlantFromUrdf();
};
