#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
 
#include <iostream>
 
int main(int argc, char ** argv)
{
  const std::string urdf_filename = argv[1];
  
  // Load the urdf model
	pinocchio::Model model;
  pinocchio::urdf::buildModel(
			urdf_filename, pinocchio::JointModelFreeFlyer(), model
			); // Specify JointModelFreeFlyer to make root joint floating base
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

	pinocchio::computeAllTerms(model, data, q, v);

	std::cout << "Joint space inertia matrix:\n";
	std::cout << data.M << std::endl << std::endl;

	std::cout << "Coriolis matrix:\n";
	std::cout << data.C << std::endl << std::endl;

	std::cout << "Gravity vector:\n";
	std::cout << data.g << std::endl;
  
} 
