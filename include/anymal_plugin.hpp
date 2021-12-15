#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include <Eigen/Core>

namespace gazebo
{
  class AnymalPlugin: public ModelPlugin
  {
    public:
			AnymalPlugin();
			~AnymalPlugin();

			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
			void SetJointVelocity(const double &_vel);
			void SetJointPosition(const double &_pos);
			double GetJointPosition();

			Eigen::Matrix<double,6,1> GetBasePose();

		private:
			physics::ModelPtr model;
			physics::WorldPtr world;
			physics::LinkPtr base;
			physics::JointPtr joint;

			std::string model_name;

			std::unique_ptr<ros::NodeHandle> rosNode;

			// Subscriptions
			ros::Subscriber velCmdSub;
			ros::Subscriber posCmdSub;

			// Advertisements
			ros::Publisher genCoordPub;

			ros::CallbackQueue rosProcessQueue;
			ros::CallbackQueue rosPublishQueue;

			std::thread rosProcessQueueThread;
			std::thread rosPublishQueueThread;

			void InitJointControllers();

			void InitRosTopics();
			void PublishQueueThread();
			void ProcessQueueThread();

			void OnVelMsg(const std_msgs::Float32ConstPtr &_msg);
			void OnPosMsg(const std_msgs::Float32ConstPtr &_msg);

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(AnymalPlugin)
}
