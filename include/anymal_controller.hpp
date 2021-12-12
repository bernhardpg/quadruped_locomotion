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
  class AnymalController: public ModelPlugin
  {
    public:
			AnymalController();

			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

			Eigen::Matrix<double,6,1> GetBasePose();

		private:
			physics::ModelPtr model;
			physics::WorldPtr world;
			physics::LinkPtr base;

			std::string model_name;

			/// \brief A node use for ROS transport
			std::unique_ptr<ros::NodeHandle> rosNode;
			/// \brief A ROS subscriber
			ros::Subscriber rosSub;
			/// \brief A ROS callbackqueue that helps process messages
			ros::CallbackQueue rosQueue;
			/// \brief A thread the keeps running the rosQueue
			std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(AnymalController)
}
