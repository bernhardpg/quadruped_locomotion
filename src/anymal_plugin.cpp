#include "anymal_plugin.hpp"

// TODO: Replace all 12 with a constant?

namespace gazebo
{
	AnymalPlugin::AnymalPlugin(){};

	// Destructor
	AnymalPlugin::~AnymalPlugin()
	{
			this->ros_node_->shutdown();

			this->ros_process_queue_.clear();
			this->ros_process_queue_.disable();
			this->ros_process_queue_thread_.join();

			this->ros_publish_queue_.clear();
			this->ros_publish_queue_.disable();
			this->ros_publish_queue_thread_.join();
	}

	void AnymalPlugin::Load(
			physics::ModelPtr model, sdf::ElementPtr sdf
			)
	{
		// Safety check
		if (model->GetJointCount() == 0)
		{
			std::cerr << "Invalid joint count, ANYmal plugin not loaded\n";
			return;
		}

		this->model_ = model;
		this->model_name_ = model->GetName();
		this->base_ = model->GetLink(this->model_name_ + "::anymal::base");

		std::cerr << "\nThe plugin is attach to model[" <<
			this->model_name_ << "]\n";

		this->world_ = model->GetWorld();

		this->joints_ = model->GetJointController()->GetJoints();
		for(auto it = this->joints_.begin(); it != this->joints_.end(); ++it)
			this->joint_names_.push_back(it->first);

		this->InitJointControllers();
		this->InitRosTopics();
	}

	void AnymalPlugin::InitJointControllers()
	{
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
		{
			this->model_->GetJointController()->SetPositionPID(
				this->joint_names_[i],
				common::PID(1000, 0.01, 10)
			);

			this->model_->GetJointController()->SetVelocityPID(
				this->joint_names_[i],
				common::PID(10, 0, 0)
				);
		}
	}

	void AnymalPlugin::SetJointVelocity(
			const std::string &joint_name, const double &vel
			)
	{
		// Set the joint's target velocity.
		this->model_->GetJointController()->SetVelocityTarget(
				joint_name, vel
				);
	} 

	void AnymalPlugin::SetJointPosition(
			const std::string &joint_name, const double &pos
			)
	{
		this->model_->GetJointController()->SetPositionTarget(
				joint_name, pos
				);
	}

	void AnymalPlugin::SetJointTorque(
			const std::string &joint_name, const double &tau
			)
	{
		this->model_->GetJointController()->SetForce(
				joint_name, tau
				);
	}

	void AnymalPlugin::SetJointPositions(
			const std::vector<double> &pos_cmds
			)
	{
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
			this->SetJointPosition(this->joint_names_[i], pos_cmds[i]);
	}

	void AnymalPlugin::SetJointVelocities(
			const std::vector<double> &vel_cmds
			)
	{
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
			this->SetJointVelocity(this->joint_names_[i], vel_cmds[i]);
	}

	void AnymalPlugin::SetJointTorques(
			const std::vector<double> &tau_cmds
			)
	{
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
		{
			this->SetJointTorque(this->joint_names_[i], tau_cmds[i]);
		}
	}

	double AnymalPlugin::GetJointPosition(const std::string &joint_name)
	{
		double pos = this->joints_[joint_name]->Position();	
		return pos;
	}

	double AnymalPlugin::GetJointVelocity(const std::string &joint_name)
	{
		double vel = this->joints_[joint_name]->GetVelocity(0);	
		return vel;
	}

	double AnymalPlugin::GetJointTorque(const std::string &joint_name)
	{
		// TODO: Gazebo API docs says that this is not yet implemented? May cause trouble!
		double torque = this->joints_[joint_name]->GetForce(0);	
		return torque;
	}

	Eigen::Matrix<double,12,1> AnymalPlugin::GetJointPositions()
	{
		Eigen::Matrix<double,12,1> q_j;
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
		{
			q_j(i) = this->GetJointPosition(this->joint_names_[i]);
		}

		return q_j;
	}

	Eigen::Matrix<double,12,1> AnymalPlugin::GetJointVelocities()
	{
		Eigen::Matrix<double,12,1> v_j;
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
		{
			v_j(i) = this->GetJointVelocity(this->joint_names_[i]);
		}

		return v_j;
	}

	Eigen::Matrix<double,12,1> AnymalPlugin::GetJointTorques()
	{
		Eigen::Matrix<double,12,1> tau_j;
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
		{
			tau_j(i) = this->GetJointTorque(this->joint_names_[i]);
		}

		return tau_j;
	}

	Eigen::Matrix<double,6,1> AnymalPlugin::GetBasePose()
	{
		ignition::math::Pose3d base_pose = this->base_->WorldPose();			

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
		this->ros_node_.reset(new ros::NodeHandle("gazebo_client"));

		// Set up advertisements
		ros::AdvertiseOptions gen_coord_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_->GetName() + "/gen_coord",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		ros::AdvertiseOptions gen_vel_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_->GetName() + "/gen_vel",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		ros::AdvertiseOptions joint_torques_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_->GetName() + "/joint_torques",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		this->gen_coord_pub_ = this->ros_node_->advertise(gen_coord_ao);
		this->gen_vel_pub_ = this->ros_node_->advertise(gen_vel_ao);
		this->joint_torques_pub_ = this->ros_node_->advertise(joint_torques_ao);

		// Set up subscriptions
		ros::SubscribeOptions vel_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_->GetName() + "/vel_cmd",
					1,
					boost::bind(&AnymalPlugin::OnVelCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		ros::SubscribeOptions pos_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_->GetName() + "/pos_cmd",
					1,
					boost::bind(&AnymalPlugin::OnPosCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		ros::SubscribeOptions torque_cmd_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_->GetName() + "/torque_cmd",
					1,
					boost::bind(&AnymalPlugin::OnTorqueCmdMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		this->vel_cmd_sub_ = this->ros_node_->subscribe(vel_cmd_so);
		this->pos_cmd_sub_ = this->ros_node_->subscribe(pos_cmd_so);
		this->torque_cmd_sub_ = this->ros_node_->subscribe(torque_cmd_so);

		// TODO: Move these to their own functions?
		// Spin up the queue helper threads
		this->ros_process_queue_thread_ = std::thread(
				std::bind(&AnymalPlugin::ProcessQueueThread, this)
				);
		this->ros_publish_queue_thread_ = std::thread(
				std::bind(&AnymalPlugin::PublishQueueThread, this)
				);
	}

	void AnymalPlugin::ProcessQueueThread()
	{
		static const double timeout = 0.01;
		while (this->ros_node_->ok())
		{
			this->ros_process_queue_.callAvailable(ros::WallDuration(timeout));
		}
	}

	void AnymalPlugin::PublishQueueThread()
	{
		ros::Rate loop_rate(200); // TODO: Is this fast enough?


		while (this->ros_node_->ok())
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

			this->gen_coord_pub_.publish(gen_coord_msg);
			this->gen_vel_pub_.publish(gen_vel_msg);
			this->joint_torques_pub_.publish(joint_torques_msg);

			loop_rate.sleep();	
		}
	}

	void AnymalPlugin::OnVelCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		this->SetJointVelocities(msg->data);
	}

	void AnymalPlugin::OnPosCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		this->SetJointPositions(msg->data);
	}

	void AnymalPlugin::OnTorqueCmdMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		this->SetJointTorques(msg->data);
	}
}
