#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
 
#include <iostream>
 
int main(int argc, char ** argv)
{
  const std::string urdf_filename = argv[1];
  
  // Load the urdf model
	pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  std::cout << "model name: " << model.name << std::endl;
  
  // Create data required by the algorithms
	pinocchio::Data data(model);
  
  // Sample a random configuration
  Eigen::VectorXd q = pinocchio::randomConfiguration(model);
  std::cout << "q: " << q.transpose() << std::endl;
 
	pinocchio::forwardKinematics(model,data,q);
 
  // Print out the placement of each joint of the kinematic tree
  for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex) model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "
              << std::fixed << std::setprecision(2)
              << data.oMi[joint_id].translation().transpose()
              << std::endl;
} 
