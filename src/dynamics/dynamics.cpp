#include "dynamics/dynamics.hpp"

// TODO: Clean up when the different terms are computed. For efficiency, this should only be done once per update.
// TODO: Clean up this class!

Dynamics::Dynamics()
{
	BuildPlantFromUrdf();
}

void Dynamics::BuildPlantFromUrdf()
{
	drake::systems::DiagramBuilder<double> builder;
	scene_graph_ = std::unique_ptr<drake::geometry::SceneGraph<double>>
		(builder.AddSystem<drake::geometry::SceneGraph>());
  scene_graph_->set_name("scene_graph"); 

	plant_ = std::unique_ptr<drake::multibody::MultibodyPlant<double>>
		(builder.AddSystem<drake::multibody::MultibodyPlant<double>>(1e-3));
  plant_->set_name("plant");
  //plant_->RegisterAsSourceForSceneGraph(&scene_graph_);

	drake::multibody::Parser parser(plant_.get()); // Pass a raw pointer
	// TODO: Load URDF file from somewhere else
	const std::string kAnymalPath =
		"drake/anymal_c_simple_description/urdf/anymal.urdf";
  const std::string urdf_path = drake::FindResourceOrThrow(kAnymalPath);
	drake::multibody::ModelInstanceIndex plant_model_instance_index
		= parser.AddModelFromFile(urdf_path);

  (void)plant_model_instance_index; // TODO: What does this do?

	plant_->Finalize();

	diagram_ =
		std::unique_ptr<drake::systems::Diagram<double>>
		(builder.Build());
	diagram_context_ = diagram_->CreateDefaultContext();

	context_ = std::unique_ptr<drake::systems::Context<double>>
		(&diagram_->GetMutableSubsystemContext(
				*plant_, diagram_context_.get())
		 );
}

// ******** //
// DYNAMICS //
// ******** //

void Dynamics::SetState(
		Eigen::Matrix<double,kNumGenCoords, 1> q,
		Eigen::Matrix<double,kNumGenVels, 1> u
		)
{
	plant_->SetPositions(context_.get(), q);
	plant_->SetVelocities(context_.get(), u);
}

Eigen::VectorXd Dynamics::GetBiasVector()
{
	Eigen::VectorXd Cv(kNumGenVels);
	plant_->CalcBiasTerm(*context_, &Cv);
	return Cv;
}

// TODO: Old pinocchio code
// Assumes that pinocchio::forwardKinematics(model_, data_, q, u, 0*u)
// has already been called
//Eigen::VectorXd Dynamics::GetContactAcc(
//		Eigen::Matrix<double,kNumGenCoords,1> q,
//		Eigen::Matrix<double,kNumGenVels,1> u,
//		int foot_i
//		)
//{
//	Eigen::VectorXd J_dot_u =
//		pinocchio::getFrameClassicalAcceleration(
//			model_, data_, model_.getFrameId(kFeetFrames[foot_i]), 
//			pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED
//			).toVector_impl();
//
//	return J_dot_u;
//}
//
//// TODO: For now this assumes that all legs are in contact
//Eigen::VectorXd Dynamics::GetContactAccPosStacked(
//		Eigen::Matrix<double,kNumGenCoords,1> q,
//		Eigen::Matrix<double,kNumGenVels,1> u
//		)
//{
//	int num_contacts = 4;
//
//	// Set u_dot = 0 to calculate J_dot * u:
//	// p_ddot = J * u_dot + J_dot * u
//	//				= J_dot * u
//	pinocchio::forwardKinematics(model_, data_, q, u, 0 * u);
//
//	Eigen::MatrixXd J_dot_u_pos(num_contacts * kNumPosDims, 1);
//	J_dot_u_pos.setZero();
//
//	for (int foot_i = 0; foot_i < kNumLegs; ++foot_i)
//	{
//		Eigen::VectorXd J_dot_u_pos_i_pos = GetContactAcc(q, u, foot_i)
//			.block<kNumPosDims,1>(0,0);
//
//		J_dot_u_pos.block<kNumPosDims,1>(kNumPosDims * foot_i,0)
//			= J_dot_u_pos_i_pos;
//	}
//	return J_dot_u_pos;
//}

// ****************** //
// FORWARD KINEMATICS // 
// ****************** //

//void Dynamics::PrintJointPlacements(
//		Eigen::VectorXd q
//		)
//{
//	pinocchio::forwardKinematics(model_, data_, q);
//
//	// Print out the placement of each joint of the kinematic tree
//	for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex) model_.njoints; ++joint_id)
//	{
//		std::cout << std::setw(24) << std::left
//							<< model_.names[joint_id] << ": "
//							<< std::fixed << std::setprecision(2)
//							<< data_.oMi[joint_id].translation().transpose()
//							<< std::endl;
//	}
//}
//
//Eigen::Vector3d Dynamics::GetFootPosInB(
//		Eigen::VectorXd q, int foot_i
//		)
//{
//	Eigen::VectorXd default_pose(7); // TODO: clean up
//	default_pose << 0, 0, 0, 0, 0, 0, 1;
//	q.block(0,0,7,1) = default_pose;
//
//	pinocchio::forwardKinematics(model_, data_, q);
//	pinocchio::updateFramePlacements(model_, data_);
//
//	Eigen::Vector3d r_B =
//		data_.oMf[model_.getFrameId(kFeetFrames[foot_i])]
//		.translation().transpose();
//	return r_B;
//}
//
//void Dynamics::PrintFootPlacement(
//		Eigen::VectorXd q, int foot_i
//		)
//{
//	auto pos = GetFootPosInB(q,foot_i);
//	std::cout << "Foot: " << kFeetFrames[foot_i] << " pos: \n";
//	PrintMatrix(pos.transpose());
//
//	auto frame = model_.frames[model_.getFrameId(kFeetFrames[foot_i])];
//	auto parent_id = frame.parent;
//	std::cout << "Parent: " << model_.names[parent_id] << "\n";
//}
//
//
//Eigen::Matrix<double,kNumFeetCoords,1> Dynamics::GetFeetPositions(
//		Eigen::Matrix<double,kNumGenCoords, 1> q
//		)
//{
//	pinocchio::forwardKinematics(model_, data_, q);
//	pinocchio::updateFramePlacements(model_, data_);
//
//	Eigen::Matrix<double,kNumFeetCoords,1> feet_positions; // LF LH RF RH
//
//	for (int foot_i = 0; foot_i < kFeetFrames.size(); ++foot_i)
//	{
//		Eigen::Vector3d pos =
//			data_.oMf[model_.getFrameId(kFeetFrames[foot_i])]
//			.translation().transpose();
//		feet_positions.block<kNumPosDims,1>(foot_i * kNumPosDims, 0) = pos;
//	}
//
//	return feet_positions;
//}

// *********************** //
// DIFFERENTIAL KINEMATICS // 
// *********************** //

Eigen::MatrixXd Dynamics::GetContactJacobianInW(int foot_i)
{
	const auto &foot_frame =
		plant_->GetBodyByName(kFeetFrames[foot_i]).body_frame();
	const auto &W_frame = plant_->world_frame();

	Eigen::MatrixXd J_c(kNumPosDims,kNumGenVels);
	plant_->CalcJacobianTranslationalVelocity(
			*context_, drake::multibody::JacobianWrtVariable::kV,
			foot_frame, Eigen::VectorXd::Zero(3),
			W_frame, W_frame, &J_c
			);

	return J_c;
}

// Drake ordering:
// [LF_HAA RF_HAA LH_HAA RH_HAA ...
//  LF_HFE RF_HFE LH_HFE RH_HFE ...
//  LF_KFE RF_KFE LH_KFE RF_KFE]
// Anymal ordering
// [LF_HAA LF_HFE LF_KFE ...
//	LH_HAA LH_HFE LH_KFE ...
//  RF_HAA RF_HFE RF_KFE ...
//	RH_HAA RH_HFE RH_KFE]

Eigen::MatrixXd Dynamics::GetStackedContactJacobianInW()
		// TODO: For now this assumes that all legs are in contact
{
	int num_contacts = 4; // TODO: replace
	Eigen::MatrixXd J_c(num_contacts * kNumPosDims,kNumGenVels);
	J_c.setZero();

	for (int foot_i = 0; foot_i < kNumLegs; ++foot_i) // TODO: change
	{
		Eigen::MatrixXd J_c_foot_i = GetContactJacobianInW(foot_i);
		J_c.block<kNumPosDims,kNumGenVels>(foot_i * kNumPosDims,0)
			= J_c_foot_i;
	}
	return J_c;
}

//Eigen::MatrixXd Dynamics::GetBaseJacobian(
//		Eigen::Matrix<double,kNumGenCoords,1> q
//		)
//{
//	// TODO: Compute jacobians in a single pass
//	Eigen::MatrixXd J_b(kNumTwistCoords,kNumGenVels);
//	J_b.setZero();
//	pinocchio::computeFrameJacobian(
//			model_, data_, q, model_.getFrameId("base"), J_b 
//			); 
//
//	return J_b;
//}
//
//Eigen::MatrixXd Dynamics::GetContactJacobian(
//		Eigen::Matrix<double,kNumGenCoords,1> q, int foot_i
//		)
//{
//	Eigen::MatrixXd J(kNumPosDims,kNumGenVels);
//	J.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3);
//	pinocchio::forwardKinematics(model_, data_, q);
//
//	const int base_id = 1;
//	Eigen::MatrixXd C_IB = data_.oMi[base_id].rotation();
//
//	Eigen::VectorXd r_B = GetFootPosInB(q, foot_i);
//	Eigen::MatrixXd r_B_skew(3,3);
//	r_B_skew << 0, -r_B(2), r_B(1),
//							r_B(2), 0, -r_B(0),
//							-r_B(1), r_B(0), 0;
//	J.block(0,3,3,3) = -C_IB * r_B_skew;
//	J.block(0,6,3,12) = C_IB * GetFootJacobianInB(q,foot_i);
//
//	std::cout << "Contact Jacobian:\n";
//	PrintMatrix(J);
//	return J;
//}
//
//Eigen::MatrixXd Dynamics::TestContactJacobian(
//		Eigen::Matrix<double,kNumGenCoords,1> q, int foot_i
//		)
//{
//
//	Eigen::VectorXd q_neutral = pinocchio::neutral(model_);
//	q_neutral(3) = 1;
//	std::cout << "q_neutral:\n";
//	PrintMatrix(q_neutral.transpose());
//
//	pinocchio::forwardKinematics(model_, data_, q_neutral);
//	
//	Eigen::Matrix3d base_attitude = data_.oMi[1].rotation(); // base
//	std::cout << "base_attitude: \n" << base_attitude << std::endl;
//	auto pos = GetFootPosInB(q_neutral, foot_i);
//	Eigen::MatrixXd pos_skew(3,3);
//	pos_skew << 0, -pos(2), pos(1),
//							pos(2), 0, -pos(0),
//							-pos(1), pos(0), 0;
//
//	std::cout << "foot pos:\n"
//		<< pos << std::endl;
//
//	std::cout << "-base_attitude*foot_skew: \n"
//		<< -base_attitude * pos_skew << std::endl;
//
//	PrintJointPlacements(q_neutral);
//	Eigen::MatrixXd J_c(kNumTwistCoords,kNumGenVels);
//	J_c.setZero();
//	pinocchio::computeFrameJacobian(
//			model_, data_, q_neutral, model_.getFrameId(kFeetFrames[foot_i]), 
//			pinocchio::ReferenceFrame::WORLD, J_c
//			); 
//
//	std::cout << "Local:\n";
//	J_c.setZero();
//	pinocchio::computeFrameJacobian(
//			model_, data_, q_neutral, model_.getFrameId(kFeetFrames[foot_i]), 
//			pinocchio::ReferenceFrame::LOCAL, J_c
//			); 
//	PrintMatrix(J_c);
//
//	std::cout << "Local_world_aligned:\n";
//	J_c.setZero();
//	pinocchio::computeFrameJacobian(
//			model_, data_, q_neutral, model_.getFrameId(kFeetFrames[foot_i]), 
//			pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_c
//			); 
//
//	PrintMatrix(J_c);
//
//	return J_c;
//}
//
//
//
//
//void Dynamics::Test()
//{
//	std::cout << std::fixed;
//	std::cout << std::setprecision(2);
//
//	Eigen::VectorXd q(kNumGenCoords);
//	q << 0, 0, 0,  // fb pos
//			 1, 0, 0, 0, // fb attitude
//			 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; // joints
//
//	Eigen::VectorXd v(18);
//	v << 0, 0, 0, // fb linear vel
//		   0, 0, 0, // fb ang vel
//			 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; // joints
//
//	// Dynamics
//	pinocchio::computeAllTerms(model_, data_, q, v);
//
//	std::cout << "nframes: " << model_.nframes << std::endl;
//
//	std::cout << "frame: " << "LF_FOOT";
//	std::cout << "  index: " << model_.getFrameId("LF_FOOT") << std::endl;
//
//	pinocchio::Data::Matrix6x J(6,model_.nv);
//	J.setZero();
//	pinocchio::computeFrameJacobian(
//			model_, data_, q, model_.getFrameId("LF_FOOT"), 
//			pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J
//			);
//
//	std::cout << "Jacobian: n x m\n" << "n: " << J.rows() << " m: " << J.cols() << std::endl;
//	std::cout << J.block(0,0,6,18) << std::endl;
//
//	pinocchio::Data::Matrix6x J_world(6,model_.nv);
//	J_world.setZero();
//	pinocchio::computeFrameJacobian(
//			model_, data_, q,
//			model_.getFrameId("LF_FOOT"),
//			pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
//			J_world
//			);
//
//	std::cout << "Jacobian in world frame: n x m\n" << "n: " << J_world.rows() << " m: " << J_world.cols() << std::endl;
//	std::cout << J_world.block(0,0,6,18) << std::endl;
//
//		// Print out the placement of each joint of the kinematic tree
//		for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex) model_.njoints; ++joint_id)
//			std::cout << std::setw(24) << std::left
//								<< model_.names[joint_id] << ": "
//								<< std::fixed << std::setprecision(2)
//								<< data_.oMi[joint_id].translation().transpose()
//								<< std::endl;
//
//	if (false)
//	{
//
//		// Print out dynamics matrices
//		std::cout << "Joint space inertia matrix:\n";
//		std::cout << data_.M << std::endl << std::endl;
//
//		std::cout << "Coriolis matrix:\n";
//		std::cout << data_.C << std::endl << std::endl;
//
//		std::cout << "Gravity vector:\n";
//		std::cout << data_.g << std::endl;
//	}
//} 
//
//

//	// TODO: Drake functionality that is to be structured in functions
//	Eigen::VectorXd q = plant_->GetPositions(context);
//	q(0) = 0.9238795;
//	q(3) = 0.382683;
//	q(7) = 0.4;
//	plant_->SetPositions(&context,q);
//	Eigen::VectorXd u = plant_->GetVelocities(context);
//
//	std::cout << "Joint names:\n";
//	for (drake::multibody::JointIndex i(0); i < plant_->num_joints(); ++i)
//	{
//		std::cout << plant_->get_joint(i).name() << std::endl;
//	}
//
//	std::cout << "q:\n";
//	PrintMatrix(q.transpose());
//
//	std::cout << "u:\n";
//	PrintMatrix(u.transpose());
//
//	const auto &LF_FOOT=
//		plant_->GetBodyByName("LF_FOOT");
//
//	const auto &W = plant_->world_frame();
//	auto pose = plant_->EvalBodyPoseInWorld(context, LF_FOOT);
//	std::cout << "LF_FOOT position" << std::endl;
//	std::cout << pose.translation() << std::endl;
//
//	Eigen::MatrixXd J_c(6,18);
//	plant_->CalcJacobianSpatialVelocity(
//			context, drake::multibody::JacobianWrtVariable::kV,
//			LF_FOOT.body_frame(), Eigen::VectorXd::Zero(3),
//			W, W, &J_c
//			);
//	// Drake has the following ordering for jacobians:
//	// [LF_HAA . . . LF_HFE . . . LF_KFE . . .]
//	std::cout << "J_c:\n";
//	PrintMatrix(J_c);
//
//	if (false) // For visualizing kinematic tree
//	{
//		std::cout << plant_->GetTopologyGraphvizString();
//	}
