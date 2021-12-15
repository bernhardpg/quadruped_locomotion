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

			virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

			// Setters
			void SetJointVelocity(
					const std::string &joint_name, const double &vel
					);
			void SetJointPosition(
					const std::string &joint_name, const double &pos
					);
			void SetJointTorque(
					const std::string &joint_name, const double &tau
					);

			void SetJointPositions(const std::vector<double> &pos_cmds);
			void SetJointVelocities(const std::vector<double> &vel_cmds);
			void SetJointTorques(const std::vector<double> &tau_cmds);

			// Getters
			double GetJointPosition(const std::string &joint_name);
			double GetJointVelocity(const std::string &joint_name);
			double GetJointTorque(const std::string &joint_name);
			Eigen::Matrix<double,12,1> GetJointPositions();
			Eigen::Matrix<double,12,1> GetJointVelocities();
			Eigen::Matrix<double,12,1> GetJointTorques();

			Eigen::Matrix<double,6,1> GetBasePose();

		private:
			std::string model_name_;
			std::vector<std::string> joint_names_;	

			physics::ModelPtr model_;
			physics::WorldPtr world_;
			physics::LinkPtr base_;
			std::map<std::string, physics::JointPtr> joints_;

			std::unique_ptr<ros::NodeHandle> ros_node_;

			// Subscriptions
			ros::Subscriber vel_cmd_sub_;
			ros::Subscriber pos_cmd_sub_;
			ros::Subscriber torque_cmd_sub_;

			// Advertisements
			ros::Publisher gen_coord_pub_;
			ros::Publisher gen_vel_pub_;
			ros::Publisher joint_torques_pub_;

			// Queues
			ros::CallbackQueue ros_process_queue_;
			ros::CallbackQueue ros_publish_queue_;
			std::thread ros_process_queue_thread_;
			std::thread ros_publish_queue_thread_;

			void InitRosTopics();
			void InitJointControllers();
			
			// Publishing and subscription runs on two separete threads
			void PublishQueueThread();
			void ProcessQueueThread();

			void OnVelCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnPosCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
			void OnTorqueCmdMsg(
					const std_msgs::Float64MultiArrayConstPtr &msg
					);
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(AnymalPlugin)
}
