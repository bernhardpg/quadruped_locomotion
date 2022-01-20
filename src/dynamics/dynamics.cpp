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

//	if (false) // For visualizing kinematic tree
//	{
//		std::cout << plant_->GetTopologyGraphvizString();
//	}
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

void Dynamics::SetStateDefault()
{
	Eigen::VectorXd q(kNumGenCoords);
	q.setZero();
	q.block<4,1>(0,0) << 1,0,0,0;
	Eigen::VectorXd u(kNumGenVels);
	u.setZero();

	SetState(q,u);
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

Eigen::VectorXd Dynamics::GetStackedContactAccInW(
		std::vector<int> legs_in_contact
		)
{
	const int num_contacts = legs_in_contact.size();

	Eigen::MatrixXd J_dot_u_pos(num_contacts * k3D, 1);
	J_dot_u_pos.setZero();

	for (int i = 0; i < num_contacts; ++i)
	{
		Eigen::VectorXd J_dot_u_pos_i_pos = GetContactAccInW(legs_in_contact[i]);
		J_dot_u_pos.block<k3D,1>(k3D * i,0)
			= J_dot_u_pos_i_pos;
	}
	return J_dot_u_pos;
}

// ****************** //
// FORWARD KINEMATICS // 
// ****************** //

Eigen::VectorXd Dynamics::GetFootPosInW(int foot_i)
{
	const auto &foot_body =
		plant_->GetBodyByName(kFeetFrames[foot_i]);

	auto pose = plant_->EvalBodyPoseInWorld(*context_, foot_body);
	return pose.translation();
}

Eigen::VectorXd Dynamics::GetStackedFootPosInW(
		const std::vector<int> &feet
		)
{
	const int num_feet = feet.size();
	Eigen::VectorXd stacked_foot_pos(k3D * num_feet);
	for (int i = 0; i < num_feet; ++i)
	{
		stacked_foot_pos.block<k3D,1>(i * k3D,0) =
			GetFootPosInW(feet[i]);
	}

	return stacked_foot_pos;
}

Eigen::MatrixXd Dynamics::GetHorisontallyStackedFootPosInW(
		const std::vector<int> &feet
		)
{
	const int num_feet = feet.size();
	Eigen::MatrixXd stacked_foot_pos(k3D, num_feet);
	for (int i = 0; i < num_feet; ++i)
	{
		stacked_foot_pos.col(i) = GetFootPosInW(feet[i]);
	}

	return stacked_foot_pos;
}

Eigen::MatrixXd Dynamics::GetStacked2DFootPosInW(
		const std::vector<int> &feet
		)
{
	return GetHorisontallyStackedFootPosInW(feet).topRows(2);
}

// *********************** //
// DIFFERENTIAL KINEMATICS // 
// *********************** //

Eigen::MatrixXd Dynamics::GetContactJacobianInW(int foot_i)
{
	const auto &foot_frame =
		plant_->GetBodyByName(kFeetFrames[foot_i]).body_frame();
	const auto &W_frame = plant_->world_frame();

	Eigen::MatrixXd J_c(k3D,kNumGenVels);
	plant_->CalcJacobianTranslationalVelocity(
			*context_, drake::multibody::JacobianWrtVariable::kV,
			foot_frame, Eigen::VectorXd::Zero(3),
			W_frame, W_frame, &J_c
			);

	return J_c;
}

Eigen::MatrixXd Dynamics::GetStackedContactJacobianInW(
		std::vector<int> legs_in_contact
		)
{
	const int num_contacts = legs_in_contact.size();
	Eigen::MatrixXd J_c(num_contacts * k3D, kNumGenVels);
	J_c.setZero();

	for (int i = 0; i < num_contacts; ++i) 
	{
		Eigen::MatrixXd J_c_leg_i = GetContactJacobianInW(legs_in_contact[i]);
		J_c.block<k3D,kNumGenVels>(i * k3D,0)
			= J_c_leg_i;
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

