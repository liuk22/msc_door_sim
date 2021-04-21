#ifndef _DOOR_INFO_HH_
#define _DOOR_INFO_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo {
  /// \brief A plugin to control a Velodyne sensor.
  class DoorInfo : public ModelPlugin {
     /// \brief Constructor
    public: 
      DoorInfo() {

      }

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        // Just output a message for now
        std::cerr << "\nThe door info plugin is attached to model[" <<
          _model->GetName() << "]\n";
      }


    /// \brief A node use for ROS transport
    private: 
      std::unique_ptr<ros::NodeHandle> rosNode;
    /// \brief A ROS subscriber
      ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
      ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
      std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(DoorInfo)
}
#endif