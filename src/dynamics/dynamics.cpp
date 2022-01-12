#include "dynamics/dynamics.hpp"

// TODO: Clean up when the different terms are computed. For efficiency, this should only be done once per update.

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

Eigen::MatrixXd Dynamics::GetMassMatrix(
		Eigen::Matrix<double,kNumGenCoords, 1> q
		)
{
	pinocchio::crba(model_, data_, q); // Computes upper triangle of M
	// Make M symmetric
	data_.M.triangularView<Eigen::StrictlyLower>() =
		data_.M.transpose().triangularView<Eigen::StrictlyLower>();
	return data_.M;
}

Eigen::VectorXd Dynamics::GetBiasVector(
		Eigen::Matrix<double,kNumGenCoords, 1> q,
		Eigen::Matrix<double,kNumGenVels, 1> u
		)
{
	pinocchio::nonLinearEffects(model_, data_, q, u);
	return data_.nle;
}

void Dynamics::UpdateState()
{
	//pinocchio::computeAllTerms(model_, data_, q, v);
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

Eigen::MatrixXd Dynamics::GetBodyPosJacobian(
		Eigen::Matrix<double,kNumGenCoords,1> q
		)
{
	Eigen::MatrixXd J(kNumTwistCoords,kNumGenVels);
	J.setZero();
	pinocchio::computeFrameJacobian(
			model_, data_, q, model_.getFrameId("base"), J 
			); 
	Eigen::MatrixXd J_pos = J.block(0,0,kNumPosDims,J.cols());

	return J_pos;
}

Eigen::MatrixXd Dynamics::GetBodyRotJacobian(
		Eigen::Matrix<double,kNumGenCoords,1> q
		)
{
	Eigen::MatrixXd J(kNumTwistCoords,kNumGenVels);
	J.setZero();
	pinocchio::computeFrameJacobian(
			model_, data_, q, model_.getFrameId("base"), J 
			);
	Eigen::MatrixXd J_pos = J.block(kNumPosDims,0,kNumPosDims,J.cols());

	return J_pos;
}

Eigen::MatrixXd Dynamics::GetContactJacobian(
		Eigen::Matrix<double,kNumGenCoords,1> q, int foot_i
		)
{
	Eigen::MatrixXd J_with_floating_body(kNumTwistCoords,kNumGenVels);
	J_with_floating_body.setZero();
	pinocchio::computeFrameJacobian(
			model_, data_, q, model_.getFrameId(kFeetFrames[foot_i]), 
			pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_with_floating_body
			); 

	return J_with_floating_body;
}

Eigen::MatrixXd Dynamics::GetContactJacobianDerivative(
		Eigen::Matrix<double,kNumGenCoords,1> q,
		Eigen::Matrix<double,kNumGenVels,1> u,
		int foot_i
		)
{
	pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, u);
	
	Eigen::MatrixXd J_dot_with_floating_body(kNumTwistCoords,kNumGenVels);
	J_dot_with_floating_body.setZero();
	pinocchio::getFrameJacobianTimeVariation(
			model_, data_, model_.getFrameId(kFeetFrames[foot_i]), 
			pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
			J_dot_with_floating_body
			); 

	return J_dot_with_floating_body;
}

Eigen::MatrixXd Dynamics::GetStackedContactJacobian(
		Eigen::Matrix<double,kNumGenCoords,1> q
		)
		// TODO: For now this assumes that all legs are in contact
{
	int num_contacts = 4; // TODO: replace
	Eigen::MatrixXd J(num_contacts * kNumTwistCoords,kNumGenVels);
	J.setZero();

	for (int foot_i = 0; foot_i < kNumLegs; ++foot_i)
	{
		Eigen::MatrixXd J_foot_i = GetContactJacobian(q, foot_i);
		J.block(foot_i * kNumTwistCoords,0,kNumTwistCoords,kNumGenVels)
			= J_foot_i;
	}
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
		Eigen::MatrixXd J_foot_i = GetContactJacobian(q, foot_i);
		auto J_foot_i_pos = J_foot_i.block<kNumPosDims,kNumGenVels>(0,0);
		J.block<kNumPosDims,kNumGenVels>(kNumPosDims * foot_i,0)
			= J_foot_i_pos;
	}
	return J;
}

Eigen::MatrixXd Dynamics::GetStackedContactJacobianPosDerivative(
		Eigen::Matrix<double,kNumGenCoords,1> q,
		Eigen::Matrix<double,kNumGenVels,1> u
		)
		// TODO: For now this assumes that all legs are in contact
{
	Eigen::MatrixXd J_dot(kNumFeetCoords,kNumGenVels);
	J_dot.setZero();

	for (int foot_i = 0; foot_i < kNumLegs; ++foot_i)
	{
		Eigen::MatrixXd J_dot_foot_i =
			GetContactJacobianDerivative(q, u, foot_i);
		auto J_dot_foot_i_pos =
			J_dot_foot_i.block<kNumPosDims,kNumGenVels>(0,0);
		J_dot.block<kNumPosDims,kNumGenVels>(kNumPosDims * foot_i,0)
			= J_dot_foot_i_pos;
	}
	return J_dot;
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
