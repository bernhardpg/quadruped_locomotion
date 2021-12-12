#include "anymal_controller.hpp"

namespace gazebo
{
	AnymalController::AnymalController(){};

	void AnymalController::Load(
			physics::ModelPtr _model, sdf::ElementPtr _sdf
			)
	{
		// Safety check
		if (_model->GetJointCount() == 0)
		{
			std::cerr << "Invalid joint count, ANYmal plugin not loaded\n";
			return;
		}

		this->model = _model;
		this->model_name = _model->GetName();
		this->base = _model->GetLink(model_name + "::anymal::base");

		std::cerr << "\nThe plugin is attach to model[" <<
			this->model_name << "]\n";

		this->world = _model->GetWorld();
	}

	Eigen::Matrix<double,6,1> AnymalController::GetBasePose()
	{
		ignition::math::Pose3d base_pose = this->base->WorldPose();			

		Eigen::Matrix<double,6,1> q_b;
		q_b(0) = base_pose.X();
    q_b(1) = base_pose.Y();
    q_b(2) = base_pose.Z();
    q_b(3) = base_pose.Roll();
    q_b(4) = base_pose.Pitch();
    q_b(5) = base_pose.Yaw();

		return q_b;
	}
}
