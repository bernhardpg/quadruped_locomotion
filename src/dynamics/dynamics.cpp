#include "dynamics/dynamics.hpp"

Dynamics::Dynamics()
{
	// TODO: Load URDF file from somewhere
  urdf_filename_ = "/home/bernhardpg/catkin_ws/src/anymal_c_simple_description/urdf/anymal.urdf";

  // Load the urdf model
  pinocchio::urdf::buildModel(
			urdf_filename_, pinocchio::JointModelFreeFlyer(), model_, false 
			); // Specify JointModelFreeFlyer to make root joint floating base
	model_.names[1] = "base"; // Set name of floating base

  // Create data required by the algorithms
	data_ = pinocchio::Data(model_);
}

Eigen::Matrix<double,kNumFeetCoords,1> Dynamics::GetFeetPositions(
		Eigen::Matrix<double,kNumGenCoords, 1> q
		)
{
	pinocchio::forwardKinematics(model_, data_, q);
	pinocchio::updateFramePlacements(model_, data_);

	Eigen::Matrix<double,kNumFeetCoords,1> feet_positions; // LF LH RF RH

	for (int foot_i = 0; foot_i < kFeetFrames.size(); ++foot_i)
	{
		Eigen::Vector3d pos =
			data_.oMf[model_.getFrameId(kFeetFrames[foot_i])]
			.translation().transpose();
		feet_positions.block<kNumPosDims,1>(foot_i * kNumPosDims, 0) = pos;
	}

	return feet_positions;
}

Eigen::MatrixXd Dynamics::GetContactJacobian(
		Eigen::Matrix<double,kNumGenCoords,1> q, int foot_i
		)
{
	Eigen::MatrixXd J(kNumTwistCoords,kNumGenVels);
	J.setZero();
	pinocchio::computeFrameJacobian(
			model_, data_, q, model_.getFrameId(kFeetFrames[foot_i]), 
			pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J
			);

	return J;
}

Eigen::MatrixXd Dynamics::GetStackedContactJacobianPos(
		Eigen::Matrix<double,kNumGenCoords,1> q
		)
		// TODO: For now this assumes that all legs are in contact
{
	Eigen::MatrixXd J(kNumFeetCoords,kNumGenVels);
	J.setZero();

	for (int foot_i = 0; foot_i < kNumLegs; ++foot_i)
	{
		Eigen::MatrixXd J_i = GetContactJacobian(q, foot_i);
		auto J_i_pos = J_i.block<kNumPosDims,kNumGenVels>(0,0);
		J.block<kNumPosDims,kNumGenVels>(kNumPosDims * foot_i,0)
			= J_i_pos;
	}
	return J;
}

void Dynamics::Test()
{
	std::cout << std::fixed;
	std::cout << std::setprecision(2);

	Eigen::VectorXd q(kNumGenCoords);
	q << 0, 0, 0,  // fb pos
			 1, 0, 0, 0, // fb attitude
			 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; // joints

	Eigen::VectorXd v(18);
	v << 0, 0, 0, // fb linear vel
		   0, 0, 0, // fb ang vel
			 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; // joints

	// Dynamics
	pinocchio::computeAllTerms(model_, data_, q, v);

	std::cout << "nframes: " << model_.nframes << std::endl;

	std::cout << "frame: " << "LF_FOOT";
	std::cout << "  index: " << model_.getFrameId("LF_FOOT") << std::endl;

	pinocchio::Data::Matrix6x J(6,model_.nv);
	J.setZero();
	pinocchio::computeFrameJacobian(
			model_, data_, q, model_.getFrameId("LF_FOOT"), 
			pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J
			);

	std::cout << "Jacobian: n x m\n" << "n: " << J.rows() << " m: " << J.cols() << std::endl;
	std::cout << J.block(0,0,6,18) << std::endl;

	pinocchio::Data::Matrix6x J_world(6,model_.nv);
	J_world.setZero();
	pinocchio::computeFrameJacobian(
			model_, data_, q,
			model_.getFrameId("LF_FOOT"),
			pinocchio::ReferenceFrame::WORLD,
			J_world
			);

	std::cout << "Jacobian in world frame: n x m\n" << "n: " << J_world.rows() << " m: " << J_world.cols() << std::endl;
	std::cout << J_world.block(0,0,6,18) << std::endl;

		// Print out the placement of each joint of the kinematic tree
		for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex) model_.njoints; ++joint_id)
			std::cout << std::setw(24) << std::left
								<< model_.names[joint_id] << ": "
								<< std::fixed << std::setprecision(2)
								<< data_.oMi[joint_id].translation().transpose()
								<< std::endl;

	if (false)
	{

		// Print out dynamics matrices
		std::cout << "Joint space inertia matrix:\n";
		std::cout << data_.M << std::endl << std::endl;

		std::cout << "Coriolis matrix:\n";
		std::cout << data_.C << std::endl << std::endl;

		std::cout << "Gravity vector:\n";
		std::cout << data_.g << std::endl;
	}
} 
