#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "eigen_conversions/eigen_msg.h"
#include "std_msgs/Float64MultiArray.h"

#include <Eigen/Core>

namespace gazebo
{
  class AnymalPlugin: public ModelPlugin
  {
    public:
			AnymalPlugin();
			~AnymalPlugin();

			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

			// Setters
			void SetJointVelocity(
					const std::string &_joint_name, const double &_vel
					);
			void SetJointPosition(
					const std::string &_joint_name, const double &_pos
					);
			void SetJointTorque(
					const std::string &_joint_name, const double &_tau
					);

			void SetJointPositions(const std::vector<double> &_pos_cmds);
			void SetJointVelocities(const std::vector<double> &_vel_cmds);
			void SetJointTorques(const std::vector<double> &_tau_cmds);

			// Getters
			double GetJointPosition(const std::string &joint_name);
			double GetJointVelocity(const std::string &joint_name);
			double GetJointTorque(const std::string &joint_name);
			Eigen::Matrix<double,12,1> GetJointPositions();
			Eigen::Matrix<double,12,1> GetJointVelocities();
			Eigen::Matrix<double,12,1> GetJointTorques();

			Eigen::Matrix<double,6,1> GetBasePose();

		private:
			physics::ModelPtr model;
			physics::WorldPtr world;
			physics::LinkPtr base;

			std::map<std::string, physics::JointPtr> joints;
			std::vector<std::string> joint_names;	

			std::string model_name;

			std::unique_ptr<ros::NodeHandle> rosNode;

			// Subscriptions
			ros::Subscriber velCmdSub;
			ros::Subscriber posCmdSub;
			ros::Subscriber torqueCmdSub;

			// Advertisements
			ros::Publisher genCoordPub;
			ros::Publisher genVelPub;
			ros::Publisher jointTorquesPub;

			ros::CallbackQueue rosProcessQueue;
			ros::CallbackQueue rosPublishQueue;

			std::thread rosProcessQueueThread;
			std::thread rosPublishQueueThread;

			void InitJointControllers();

			void InitRosTopics();
			void PublishQueueThread();
			void ProcessQueueThread();

			void OnVelCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &_msg
					);
			void OnPosCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &_msg
					);
			void OnTorqueCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &_msg
					);
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(AnymalPlugin)
}
