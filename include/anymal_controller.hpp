#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

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

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(AnymalController)
}
