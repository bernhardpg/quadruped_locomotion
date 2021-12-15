#include "anymal_plugin.hpp"

namespace gazebo
{
	AnymalPlugin::AnymalPlugin(){};

	// Destructor
	AnymalPlugin::~AnymalPlugin()
	{
			this->rosNode->shutdown();

			this->rosProcessQueue.clear();
			this->rosProcessQueue.disable();
			this->rosProcessQueueThread.join();

			this->rosPublishQueue.clear();
			this->rosPublishQueue.disable();
			this->rosPublishQueueThread.join();
	}

	void AnymalPlugin::Load(
			physics::ModelPtr _model, sdf::ElementPtr _sdf
			)
	{
		// TODO: Advertise joint velocities 
		// TODO: Implement position control 
		// TODO: Somehow tune PIDs?
		// TODO: Implement velocity control
		// TODO: Generalize to all joints
		// TODO: Advertise joint positions
		// TODO: Advertise base pose

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

		// TODO: Expand to all joints
		this->joint = _model->GetJoints()[0];
		std::cout << "\nJoint name: " << this->joint->GetName() << std::endl;

		this->InitJointControllers();
		std::cout << "Initialized joint controllers\n";

		this->InitRosTopics();
		std::cout << "Initialized ros topics\n";
	}

	void AnymalPlugin::InitJointControllers()
	{
		this->model->GetJointController()->SetVelocityPID(
			this->joint->GetScopedName(),
			common::PID(1, 0, 0)
			);

		this->model->GetJointController()->SetPositionPID(
			this->joint->GetScopedName(),
			common::PID(1, 0, 0)
		);
	}

	void AnymalPlugin::SetJointVelocity(const double &_vel)
	{
		// Set the joint's target velocity.
		this->model->GetJointController()->SetVelocityTarget(
				this->joint->GetScopedName(), _vel
				);
	} 

	void AnymalPlugin::SetJointPosition(const double &_pos)
	{
		this->model->GetJointController()->SetPositionTarget(
				this->joint->GetScopedName(), _pos
				);
	}

	double AnymalPlugin::GetJointPosition()
	{
		double pos = this->joint->Position();	
		return pos;
	}

	void AnymalPlugin::InitRosTopics()
	{
		// Initialize ROS, if it has not already bee initialized.
		if (!ros::isInitialized())
		{
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client",
					ros::init_options::NoSigintHandler
					);
		}

		// Create ros node
		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

		// Set up advertisements
		ros::AdvertiseOptions gen_coord_ao =
			ros::AdvertiseOptions::create<std_msgs::Float32>(
					"/" + this->model->GetName() + "/gen_coord",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->rosPublishQueue
					);

		this->genCoordPub = this->rosNode->advertise(gen_coord_ao);

		// Set up subscriptions
		ros::SubscribeOptions vel_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float32>(
					"/" + this->model->GetName() + "/vel_cmd",
					1,
					boost::bind(&AnymalPlugin::OnVelMsg, this, _1),
					ros::VoidPtr(),
					&this->rosProcessQueue
					);

		ros::SubscribeOptions pos_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float32>(
					"/" + this->model->GetName() + "/pos_cmd",
					1,
					boost::bind(&AnymalPlugin::OnPosMsg, this, _1),
					ros::VoidPtr(),
					&this->rosProcessQueue
					);

		this->velCmdSub = this->rosNode->subscribe(vel_cmd_so);
		this->posCmdSub = this->rosNode->subscribe(pos_cmd_so);

		// TODO: Move these to their own functions?
		// Spin up the queue helper threads
		this->rosProcessQueueThread = std::thread(
				std::bind(&AnymalPlugin::ProcessQueueThread, this)
				);
		this->rosPublishQueueThread = std::thread(
				std::bind(&AnymalPlugin::PublishQueueThread, this)
				);
	}

	void AnymalPlugin::ProcessQueueThread()
	{
		static const double timeout = 0.01;
		while (this->rosNode->ok())
		{
			this->rosProcessQueue.callAvailable(ros::WallDuration(timeout));
		}
	}

	void AnymalPlugin::PublishQueueThread()
	{
		ros::Rate loop_rate(200); // TODO: Is this fast enough?

		while (this->rosNode->ok())
		{
			// TODO: Expand to all joints
			std_msgs::Float32 gen_coord_msg;
			gen_coord_msg.data = this->GetJointPosition();
			this->genCoordPub.publish(gen_coord_msg);

			loop_rate.sleep();	
		}
	}

	void AnymalPlugin::OnVelMsg(const std_msgs::Float32ConstPtr &_msg)
	{
		this->SetJointVelocity(_msg->data);
	}

	void AnymalPlugin::OnPosMsg(const std_msgs::Float32ConstPtr &_msg)
	{
		this->SetJointPosition(_msg->data);
	}

	Eigen::Matrix<double,6,1> AnymalPlugin::GetBasePose()
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
