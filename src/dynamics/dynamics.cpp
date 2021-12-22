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

	// Reflects URDF
	feet_frames_.push_back("LF_FOOT");
	feet_frames_.push_back("LH_FOOT");
	feet_frames_.push_back("RF_FOOT");
	feet_frames_.push_back("RH_FOOT");

  // Create data required by the algorithms
	data_ = pinocchio::Data(model_);

  std::cout << "Loaded dynamics with model name: "
		<< model_.name << std::endl;
	std::cout << "	nq: " << model_.nq << std::endl;
	std::cout << "	nv: " << model_.nv << std::endl;
}

Eigen::Matrix<double,12,1> Dynamics::GetFeetPositions(
		Eigen::Matrix<double, 19, 1> q
		)
{
	pinocchio::forwardKinematics(model_, data_, q);
	pinocchio::updateFramePlacements(model_, data_);

	Eigen::Matrix<double,12,1> feet_positions; // LF LH RF RH

	for (int foot_i = 0; foot_i < feet_frames_.size(); ++foot_i)
	{
		Eigen::Vector3d pos = data_.oMf[model_.getFrameId(feet_frames_[foot_i])]
			.translation().transpose();
		feet_positions.block<3,1>(foot_i * 3, 0) = pos;
	}

	return feet_positions;
}

Eigen::MatrixXd Dynamics::GetFootJacobian(
		Eigen::Matrix<double,19,1> q, int foot_i
		)
{
	Eigen::MatrixXd J(6, model_.nv);
	J.setZero();
	pinocchio::computeFrameJacobian(
			model_, data_, q, model_.getFrameId(feet_frames_[foot_i]), 
			pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J
			);

	return J;
}

// TODO: Currently returns stacked jacobian for all feet
Eigen::MatrixXd Dynamics::GetFullContactJacobian(
		Eigen::Matrix<double,19,1> q
		)
{
	Eigen::MatrixXd J(24, model_.nv);
	J.setZero();

	for (int foot_i = 0; foot_i < n_legs_; ++foot_i)
	{
		Eigen::MatrixXd J_i = GetFootJacobian(q, foot_i);
		J.block<6,18>(6 * foot_i,0) = J_i;
	}

	return J;
}

void Dynamics::Test()
{
	std::cout << std::fixed;
	std::cout << std::setprecision(2);

	Eigen::VectorXd q(19);
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
