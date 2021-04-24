#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <sdf/sdf.hh>
#include <sensor_msgs/JointState.h>

namespace gazebo {
  class DoorInfo : public ModelPlugin {

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    public: 
      DoorInfo() {

      }

      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        // Just output a message for now


        this->model = _model; 
        this->sdf = _sdf; 
        this->world = _model->GetWorld();
        this->updateRate = 20.0;
        for (physics::JointPtr jointptr : this->model->GetJoints()) {
          if (jointptr->GetName().compare("hinge") == 0) {
            this->hinge = jointptr;
            break;
          }
        }

        this->rosNode = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("hinged_door_NodeHandle"));
        this->pub = this->rosNode->advertise<sensor_msgs::JointState>("door_info", 1000);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                                  boost::bind (&DoorInfo::OnUpdate, this)
                                );
        this->lastUpdateTime = this->world->GetSimTime(); 
        std::cerr << "\nThe door info plugin has successfully attached\n";
      }

      void OnUpdate() { 
        common::Time curTime = world->GetSimTime();
        double secondsPassedTime = (curTime - this->lastUpdateTime).Double();
        if ( secondsPassedTime > 1.0 / this->updateRate ) {
          ros::Time rosTime = ros::Time::now();
          this->hingeState.header.stamp = rosTime;
          double velocity = this->hinge->GetVelocity(0);
          double position = this->hinge->GetAngle(0).Radian();

          this->hingeState.name[0] = "hinge";
          this->hingeState.position[0] = position;
          this->hingeState.velocity[0] = velocity;

          this->lastUpdateTime += common::Time (1.0 / this->updateRate);
          this->pub.publish(this->hingeState);
        }
      }

    private: 
      physics::ModelPtr model;
      sdf::ElementPtr sdf; 
      physics::WorldPtr world;
      physics::JointPtr hinge;
      sensor_msgs::JointState hingeState;
      
      boost::shared_ptr<ros::NodeHandle> rosNode;
      event::ConnectionPtr updateConnection;
      ros::Publisher pub; 

      double updateRate;
      common::Time lastUpdateTime;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(DoorInfo)
}