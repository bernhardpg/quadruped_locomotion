#include "gazebo/anymal_plugin.hpp"

// TODO: Replace all unecessary instances of this

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
		this->base_ = model->GetLink(this->model_name_ + "::base");

		std::cerr << "\nThe plugin is attach to model[" <<
			this->model_name_ << "]\n";

		this->world_ = model_->GetWorld();

		this->joints_ = model_->GetJointController()->GetJoints();
		for(auto it = this->joints_.begin(); it != this->joints_.end(); ++it)
			this->joint_names_.push_back(it->first);

		// Print joint order for convenience
		for (auto it = joint_names_.begin(); it != joint_names_.end(); ++it)
		{
			std::cout << *it << std::endl;
		}

		this->InitRosTopics();
    if (!LoadParametersFromRos())
    {
        ROS_ERROR("Could not load parameters.");
        return;
    }
		this->InitJointControllers();
	}

	bool AnymalPlugin::ResetSimulation(
					const std_srvs::Empty::Request &_req,
					std_srvs::Empty::Response &_res
			)
	{
		world_->Reset();
		model_->GetJointController()->Reset();
		return true;
	}

	void AnymalPlugin::InitJointControllers()
	{
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
		{
			this->model_->GetJointController()->SetPositionPID(
				this->joint_names_[i],
				common::PID(pos_p_gain_, pos_i_gain_, pos_d_gain_) 
			);

			this->model_->GetJointController()->SetVelocityPID(
				this->joint_names_[i],
				common::PID(vel_p_gain_, vel_i_gain_, vel_d_gain_)
				);
		}
	}

	bool AnymalPlugin::LoadParametersFromRos()
	{
		if (!this->ros_node_->getParam(
					"joint_position_controller/p_gain",
					this->pos_p_gain_))
    {
        ROS_ERROR("Could not read position P gains from parameter server.");
        return false;
    }

		if (!this->ros_node_->getParam(
					"joint_position_controller/i_gain",
					this->pos_i_gain_))
    {
        ROS_ERROR("Could not read position I gains from parameter server.");
        return false;
    }

		if (!this->ros_node_->getParam(
					"joint_position_controller/d_gain",
					this->pos_d_gain_))
    {
        ROS_ERROR("Could not read position D gains from parameter server.");
        return false;
    }

		if (!this->ros_node_->getParam(
					"joint_velocity_controller/p_gain",
					this->vel_p_gain_))
    {
        ROS_ERROR("Could not read velocity P gains from parameter server.");
        return false;
    }

		if (!this->ros_node_->getParam(
					"joint_velocity_controller/i_gain",
					this->vel_i_gain_))
    {
        ROS_ERROR("Could not read velocity I gains from parameter server.");
        return false;
    }

		if (!this->ros_node_->getParam(
					"joint_velocity_controller/d_gain",
					this->vel_d_gain_))
    {
        ROS_ERROR("Could not read velocity D gains from parameter server.");
        return false;
    }
		return true;
	}

	bool AnymalPlugin::UpdateParameterFromRos(
			std::string param_name, double *param_ptr)
	{
		double param_value;
		ros_node_->getParam(param_name, param_value);
		if (param_value != *param_ptr)
		{
			*param_ptr = param_value;	
			return true;
		}
		return false;
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

	// TODO: Note that position control is poorly tuned as it not used
	void AnymalPlugin::SetJointPositions(
			const std::vector<double> &pos_cmds
			)
	{
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
			this->SetJointPosition(this->joint_names_[i], pos_cmds[i]);
	}

	// TODO: Note that velocity control is poorly tuned as it not used
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

	joint_vector_t AnymalPlugin::GetJointPositions()
	{
		joint_vector_t q_j;
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
		{
			q_j(i) = this->GetJointPosition(this->joint_names_[i]);
		}

		return q_j;
	}

	joint_vector_t AnymalPlugin::GetJointVelocities()
	{
		joint_vector_t v_j;
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
		{
			v_j(i) = this->GetJointVelocity(this->joint_names_[i]);
		}

		return v_j;
	}

	joint_vector_t AnymalPlugin::GetJointTorques()
	{
		joint_vector_t tau_j;
		for (size_t i = 0; i < this->joint_names_.size(); ++i)
		{
			tau_j(i) = this->GetJointTorque(this->joint_names_[i]);
		}

		return tau_j;
	}

	Eigen::Matrix<double, 7, 1> AnymalPlugin::GetBasePose()
	{
		ignition::math::Pose3d base_pose = this->base_->WorldPose();			

		Eigen::Matrix<double, 7, 1> q_b;
		q_b(0) = base_pose.Pos().X();
		q_b(1) = base_pose.Pos().Y();
		q_b(2) = base_pose.Pos().Z();
		q_b(6) = base_pose.Rot().W();
		q_b(3) = base_pose.Rot().X();
		q_b(4) = base_pose.Rot().Y();
		q_b(5) = base_pose.Rot().Z();

		return q_b;
	}

	Eigen::Matrix<double, 6, 1> AnymalPlugin::GetBaseTwist()
	{
		ignition::math::Vector3d base_linear_velocity =
			this->base_->WorldLinearVel();			
		ignition::math::Vector3d base_angular_velocity =
			this->base_->WorldAngularVel();			

		Eigen::VectorXd u_b(6);
		u_b(0) = base_linear_velocity.X();
		u_b(1) = base_linear_velocity.Y();
		u_b(2) = base_linear_velocity.Z();
		u_b(3) = base_angular_velocity.X();
		u_b(4) = base_angular_velocity.Y();
		u_b(5) = base_angular_velocity.Z();

		return u_b;
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

		// Set up services
		ros::AdvertiseServiceOptions reset_simulation_aso =
			ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
            "/" + model_->GetName() + "/reset_simulation",
            boost::bind(&AnymalPlugin::ResetSimulation, this, _1, _2),
            ros::VoidPtr(),
            &this->ros_process_queue_
        );

		reset_simulation_service_ = ros_node_->advertiseService(reset_simulation_aso);

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
		this->ros_param_server_thread_ = std::thread(
				std::bind(&AnymalPlugin::ParamServerThread, this)
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
		ros::Rate loop_rate(2500);


		while (this->ros_node_->ok())
		{
			Eigen::Matrix<double, 7, 1> q_b;
			q_b = GetBasePose();
			joint_vector_t q_j; // generalized coordinates
			q_j = this->GetJointPositions();

			gen_coord_vector_t q;
			q.block<7,1>(0,0) = q_b;
			q.block<kNumJoints,1>(kNumPoseCoords,0) = q_j;

			Eigen::Matrix<double, 6, 1> u_b;
			u_b = GetBaseTwist();
			joint_vector_t u_j;
			u_j = this->GetJointVelocities();
			
			gen_vel_vector_t	u;
			u.block<6,1>(0,0) = u_b;
			u.block<kNumJoints,1>(6,0) = u_j;

			joint_vector_t tau; // joint torques
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

	void AnymalPlugin::ParamServerThread()
	{
		ros::Rate loop_rate(4);

		bool updated;
		while (this->ros_node_->ok())
		{
			updated = false;
			// Velocity PID
			updated = UpdateParameterFromRos(
					"joint_velocity_controller/p_gain", &vel_p_gain_
					) || updated;
			updated = UpdateParameterFromRos(
					"joint_velocity_controller/i_gain", &vel_i_gain_
					) || updated;
			updated = UpdateParameterFromRos(
					"joint_velocity_controller/d_gain", &vel_d_gain_
					) || updated;

			// Position PID
			updated = UpdateParameterFromRos(
					"joint_position_controller/p_gain", &pos_p_gain_
					) || updated;
			updated = UpdateParameterFromRos(
					"joint_position_controller/i_gain", &pos_i_gain_
					) || updated;
			updated = UpdateParameterFromRos(
					"joint_position_controller/d_gain", &pos_d_gain_
					) || updated;

			if (updated)
			{
				InitJointControllers();
			}
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
