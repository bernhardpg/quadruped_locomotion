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
		Eigen::MatrixXd GetMassMatrix();
		Eigen::VectorXd GetBiasVector();
		Eigen::VectorXd GetContactAcc(
				Eigen::Matrix<double,kNumGenCoords,1> q,
				Eigen::Matrix<double,kNumGenVels,1> u,
				int foot_i
				);
		Eigen::VectorXd GetContactAccPosStacked(
				Eigen::Matrix<double,kNumGenCoords,1> q,
				Eigen::Matrix<double,kNumGenVels,1> u
				);

		// ****************** //
		// FORWARD KINEMATICS // 
		// ****************** //

		void PrintJointPlacements(
				Eigen::VectorXd q
				);
		void PrintFootPlacement(
				Eigen::VectorXd q, int foot_i
				);
		Eigen::Vector3d GetFootPosInB(
				Eigen::VectorXd q, int foot_i
				);
		Eigen::Matrix<double,kNumFeetCoords,1> GetFeetPositions(
				Eigen::Matrix<double,kNumGenCoords, 1> q
				);

		// *********************** //
		// DIFFERENTIAL KINEMATICS // 
		// *********************** //

		Eigen::MatrixXd GetBaseJacobian(
				Eigen::Matrix<double,kNumGenCoords,1> q
				);
		Eigen::MatrixXd GetContactJacobian(
				Eigen::Matrix<double,kNumGenCoords,1> q, int foot_i
				);
		Eigen::MatrixXd GetFootJacobianInB(
				Eigen::Matrix<double,kNumGenCoords,1> q, int foot_i
				);
		Eigen::MatrixXd TestContactJacobian( // TODO: remove
				Eigen::Matrix<double,kNumGenCoords,1> q, int foot_i
				);
		Eigen::MatrixXd GetStackedContactJacobian(
				Eigen::Matrix<double,kNumGenCoords,1> q
				);
		Eigen::MatrixXd GetStackedContactJacobianPos(
				Eigen::Matrix<double,kNumGenCoords,1> q
				);

		void Test(); // TODO: Remove

	private:
		std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_;
		std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
		std::unique_ptr<drake::systems::Context<double>> context_;
		std::unique_ptr<drake::systems::Diagram<double>> diagram_;
		std::unique_ptr<drake::systems::Context<double>> diagram_context_;

		void BuildPlantFromUrdf();

		std::string urdf_filename_;
		pinocchio::Model model_;
		pinocchio::Data data_;
};
