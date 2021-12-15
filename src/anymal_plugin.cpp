#include "anymal_plugin.hpp"

// TODO: Fix style and casing!
// TODO: Replace all 12 with a constant?

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

		this->joints = _model->GetJointController()->GetJoints();
		for(auto it = joints.begin(); it != joints.end(); ++it)
			joint_names.push_back(it->first);

		this->InitJointControllers();
		this->InitRosTopics();
	}

	void AnymalPlugin::InitJointControllers()
	{
		for (size_t i = 0; i < this->joint_names.size(); ++i)
		{
			this->model->GetJointController()->SetVelocityPID(
				this->joint_names[i],
				common::PID(10, 0, 0)
				);

			this->model->GetJointController()->SetPositionPID(
				this->joint_names[i],
				common::PID(10, 0, 0)
			);
		}
	}

	void AnymalPlugin::SetJointVelocity(
			const std::string &_joint_name, const double &_vel
			)
	{
		// Set the joint's target velocity.
		this->model->GetJointController()->SetVelocityTarget(
				_joint_name, _vel
				);
	} 

	void AnymalPlugin::SetJointPosition(
			const std::string &_joint_name, const double &_pos
			)
	{
		this->model->GetJointController()->SetPositionTarget(
				_joint_name, _pos
				);
	}

	void AnymalPlugin::SetJointTorque(
			const std::string &_joint_name, const double &_tau
			)
	{
		this->model->GetJointController()->SetForce(
				_joint_name, _tau
				);
	}

	void AnymalPlugin::SetJointPositions(
			const std::vector<double> &_pos_cmds
			)
	{
		for (size_t i = 0; i < this->joint_names.size(); ++i)
			this->SetJointTorque(this->joint_names[i], _pos_cmds[i]);
	}

	void AnymalPlugin::SetJointVelocities(
			const std::vector<double> &_vel_cmds
			)
	{
		for (size_t i = 0; i < this->joint_names.size(); ++i)
			this->SetJointTorque(this->joint_names[i], _vel_cmds[i]);
	}

	void AnymalPlugin::SetJointTorques(
			const std::vector<double> &_tau_cmds
			)
	{
		for (size_t i = 0; i < this->joint_names.size(); ++i)
		{
			this->SetJointTorque(this->joint_names[i], _tau_cmds[i]);
		}
	}

	double AnymalPlugin::GetJointPosition(const std::string &_joint_name)
	{
		double pos = this->joints[_joint_name]->Position();	
		return pos;
	}

	double AnymalPlugin::GetJointVelocity(const std::string &_joint_name)
	{
		double vel = this->joints[_joint_name]->GetVelocity(0);	
		return vel;
	}

	double AnymalPlugin::GetJointTorque(const std::string &_joint_name)
	{
		// TODO: Gazebo API docs says that this is not yet implemented? May cause trouble!
		double torque = this->joints[_joint_name]->GetForce(0);	
		return torque;
	}

	Eigen::Matrix<double,12,1> AnymalPlugin::GetJointPositions()
	{
		Eigen::Matrix<double,12,1> q_j;
		for (size_t i = 0; i < this->joint_names.size(); ++i)
		{
			q_j(i) = this->GetJointPosition(this->joint_names[i]);
		}

		return q_j;
	}

	Eigen::Matrix<double,12,1> AnymalPlugin::GetJointVelocities()
	{
		Eigen::Matrix<double,12,1> v_j;
		for (size_t i = 0; i < this->joint_names.size(); ++i)
		{
			v_j(i) = this->GetJointVelocity(this->joint_names[i]);
		}

		return v_j;
	}

	Eigen::Matrix<double,12,1> AnymalPlugin::GetJointTorques()
	{
		Eigen::Matrix<double,12,1> tau_j;
		for (size_t i = 0; i < this->joint_names.size(); ++i)
		{
			tau_j(i) = this->GetJointTorque(this->joint_names[i]);
		}

		return tau_j;
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

	void AnymalPlugin::InitRosTopics()
	{
		// Initialize ROS, if it has not already been initialized.
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
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model->GetName() + "/gen_coord",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->rosPublishQueue
					);

		ros::AdvertiseOptions gen_vel_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model->GetName() + "/gen_vel",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->rosPublishQueue
					);

		ros::AdvertiseOptions joint_torques_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model->GetName() + "/joint_torques",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->rosPublishQueue
					);

		this->genCoordPub = this->rosNode->advertise(gen_coord_ao);
		this->genVelPub = this->rosNode->advertise(gen_vel_ao);
		this->jointTorquesPub = this->rosNode->advertise(joint_torques_ao);

		// Set up subscriptions
		ros::SubscribeOptions vel_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model->GetName() + "/vel_cmd",
					1,
					boost::bind(&AnymalPlugin::OnVelCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->rosProcessQueue
					);

		ros::SubscribeOptions pos_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model->GetName() + "/pos_cmd",
					1,
					boost::bind(&AnymalPlugin::OnPosCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->rosProcessQueue
					);

		ros::SubscribeOptions torque_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model->GetName() + "/torque_cmd",
					1,
					boost::bind(&AnymalPlugin::OnTorqueCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->rosProcessQueue
					);

		this->velCmdSub = this->rosNode->subscribe(vel_cmd_so);
		this->posCmdSub = this->rosNode->subscribe(pos_cmd_so);
		this->torqueCmdSub = this->rosNode->subscribe(torque_cmd_so);

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
			// TODO: Expand with base pose, not just joints!
			Eigen::Matrix<double, 12, 1> q; // generalized coordinates
			q = this->GetJointPositions();

			Eigen::Matrix<double, 12, 1> u; // generalized velocities
			u = this->GetJointVelocities();

			Eigen::Matrix<double, 12, 1> tau; // joint torques
			tau = this->GetJointTorques();

			std_msgs::Float64MultiArray gen_coord_msg;
			tf::matrixEigenToMsg(q, gen_coord_msg);

			std_msgs::Float64MultiArray gen_vel_msg;
			tf::matrixEigenToMsg(u, gen_vel_msg);

			std_msgs::Float64MultiArray joint_torques_msg;
			tf::matrixEigenToMsg(tau, joint_torques_msg);

			this->genCoordPub.publish(gen_coord_msg);
			this->genVelPub.publish(gen_vel_msg);
			this->jointTorquesPub.publish(joint_torques_msg);

			loop_rate.sleep();	
		}
	}

	void AnymalPlugin::OnVelCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &_msg
			)
	{
		this->SetJointVelocities(_msg->data);
	}

	void AnymalPlugin::OnPosCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &_msg
			)
	{
		this->SetJointPositions(_msg->data);
	}

	void AnymalPlugin::OnTorqueCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &_msg
			)
	{
		this->SetJointTorques(_msg->data);
	}
}
