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

Eigen::MatrixXd Dynamics::GetMassMatrix()
{
	Eigen::MatrixXd M(kNumGenVels,kNumGenVels);
	plant_->CalcMassMatrix(*context_, &M);
	return M;
}

Eigen::VectorXd Dynamics::GetBiasVector()
{
	Eigen::VectorXd Cv(kNumGenVels);
	plant_->CalcBiasTerm(*context_, &Cv);
	return Cv;
}

Eigen::VectorXd Dynamics::GetContactAccInW(int foot_i)
{
	// Calculate J_dot * u as acceleration
	// p_ddot = J * u_dot + J_dot * u
	//				= J_dot * u

	const auto &foot_frame =
		plant_->GetBodyByName(kFeetFrames[foot_i]).body_frame();
	const auto &W_frame = plant_->world_frame();

	Eigen::VectorXd J_dot_u =
		plant_->CalcBiasTranslationalAcceleration(
			*context_, drake::multibody::JacobianWrtVariable::kV,
			foot_frame, Eigen::VectorXd::Zero(3),
			W_frame, W_frame);

	return J_dot_u;
}

// TODO: For now this assumes that all legs are in contact
Eigen::VectorXd Dynamics::GetStackedContactAccInW()
{
	int num_contacts = 4;

	Eigen::MatrixXd J_dot_u_pos(num_contacts * kNumPosDims, 1);
	J_dot_u_pos.setZero();

	for (int foot_i = 0; foot_i < kNumLegs; ++foot_i)
	{
		Eigen::VectorXd J_dot_u_pos_i_pos = GetContactAccInW(foot_i);
		J_dot_u_pos.block<kNumPosDims,1>(kNumPosDims * foot_i,0)
			= J_dot_u_pos_i_pos;
	}
	return J_dot_u_pos;
}

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

// TODO: the point of this is really to rotate it to the command frame
Eigen::MatrixXd Dynamics::GetBaseJacobianInW()
{
	const auto &base_frame =
		plant_->GetBodyByName("base").body_frame();
	const auto &W_frame = plant_->world_frame();

	Eigen::MatrixXd J_b(kNumTwistCoords,kNumGenVels);
	J_b.setZero();

	plant_->CalcJacobianSpatialVelocity(
		*context_, drake::multibody::JacobianWrtVariable::kV,
		base_frame, Eigen::VectorXd::Zero(3),
		W_frame, W_frame, &J_b);

	return J_b;
}

//	// TODO: Drake functionality that is to be structured in functions
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
