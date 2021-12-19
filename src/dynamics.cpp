#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include <iostream>
#include <iomanip>
 
// TODO: Factor this into a library
// TODO: Load URDF file from somewhere

int main(int argc, char ** argv)
{
	std::cout << std::fixed;
	std::cout << std::setprecision(2);

  const std::string urdf_filename = argv[1];
  
  // Load the urdf model
	pinocchio::Model model;
  pinocchio::urdf::buildModel(
			urdf_filename, pinocchio::JointModelFreeFlyer(), model, false 
			); // Specify JointModelFreeFlyer to make root joint floating base
	model.names[1] = "base"; // Set name of floating base

  std::cout << "model name: " << model.name << std::endl;
  
  // Create data required by the algorithms
	pinocchio::Data data(model);

	std::cout << "	nq: " << model.nq << std::endl;
	std::cout << "	nv: " << model.nv << std::endl;

	Eigen::VectorXd q(19);
	q << 0, 0, 0,  // fb pos
			 1, 0, 0, 0, // fb attitude
			 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; // joints

	Eigen::VectorXd v(18);
	v << 0, 0, 0, // fb linear vel
		   0, 0, 0, // fb ang vel
			 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; // joints

	// Dynamics
	pinocchio::computeAllTerms(model, data, q, v);

	std::cout << "nframes: " << model.nframes << std::endl;

	std::cout << "frame: " << "LF_FOOT";
	std::cout << "  index: " << model.getFrameId("LF_FOOT") << std::endl;

	pinocchio::Data::Matrix6x J(6,model.nv);
	J.setZero();
	pinocchio::computeFrameJacobian(
			model, data, q, model.getFrameId("LF_FOOT"), 
			pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J
			);

	std::cout << "Jacobian: n x m\n" << "n: " << J.rows() << " m: " << J.cols() << std::endl;
	std::cout << J.block(0,0,6,18) << std::endl;

	pinocchio::Data::Matrix6x J_world(6,model.nv);
	J_world.setZero();
	pinocchio::computeFrameJacobian(
			model, data, q,
			model.getFrameId("LF_FOOT"),
			pinocchio::ReferenceFrame::WORLD,
			J_world
			);

	std::cout << "Jacobian in world frame: n x m\n" << "n: " << J_world.rows() << " m: " << J_world.cols() << std::endl;
	std::cout << J_world.block(0,0,6,18) << std::endl;

		// Print out the placement of each joint of the kinematic tree
		for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex) model.njoints; ++joint_id)
			std::cout << std::setw(24) << std::left
								<< model.names[joint_id] << ": "
								<< std::fixed << std::setprecision(2)
								<< data.oMi[joint_id].translation().transpose()
								<< std::endl;

	if (false)
	{

		// Print out dynamics matrices
		std::cout << "Joint space inertia matrix:\n";
		std::cout << data.M << std::endl << std::endl;

		std::cout << "Coriolis matrix:\n";
		std::cout << data.C << std::endl << std::endl;

		std::cout << "Gravity vector:\n";
		std::cout << data.g << std::endl;
	}
} 
