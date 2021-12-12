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

		// Get the first joint. We are making an assumption about the model
		// having one joint that is the rotational joint.
		this->joint = _model->GetJoints()[0];

		std::cout << "\nJoint name: " << this->joint->GetName() << std::endl;

		// Setup a P-controller, with a gain of 0.1.
		this->pid = common::PID(0.1, 0, 0);

		// Apply the P-controller to the joint.
		this->model->GetJointController()->SetVelocityPID(
				this->joint->GetScopedName(), this->pid);

		this->InitRosTopics();
	}

	void AnymalController::SetVelocity(const double &_vel)
	{
		// Set the joint's target velocity.
		this->model->GetJointController()->SetVelocityTarget(
				this->joint->GetScopedName(), _vel);
	} 

	void AnymalController::InitRosTopics()
	{
		// Initialize ROS, if it has not already bee initialized.
		if (!ros::isInitialized())
		{
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client",
					ros::init_options::NoSigintHandler);
		}
		// Create ROS node
		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

		// Create a named topic, and subscribe to it.
		ros::SubscribeOptions so =
			ros::SubscribeOptions::create<std_msgs::Float32>(
					"/" + this->model->GetName() + "/vel_cmd",
					1,
					boost::bind(&AnymalController::OnRosMsg, this, _1),
					ros::VoidPtr(), &this->rosQueue);
		this->rosSub = this->rosNode->subscribe(so);

		// Spin up the queue helper thread.
		this->rosQueueThread =
			std::thread(std::bind(&AnymalController::QueueThread, this));

	}

	void AnymalController::OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
	{
		std::cout << "New ros message" << std::endl;
		this->SetVelocity(_msg->data);
	}

	/// \brief ROS helper function that processes messages
	void AnymalController::QueueThread()
	{
		static const double timeout = 0.01;
		while (this->rosNode->ok())
		{
			this->rosQueue.callAvailable(ros::WallDuration(timeout));
		}
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
