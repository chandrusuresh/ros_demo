#ifndef IMU_PLUGIN_H
#define IMU_PLUGIN_H

// #include <random>

// #include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "Imu.pb.h"

#include "rotors_gazebo_plugins/common.h"

#include "sensor.h"
#include "sensor_noise.h"

using namespace std;
using namespace ros_demo;

namespace gazebo
{
    class IMUPlugin : public ModelPlugin
    {
        public:
        IMUPlugin();
        ~IMUPlugin();
        void Publish();

        protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        void OnUpdate(const common::UpdateInfo&);
        
        private:
        bool isInit = true;
        void CreatePubsAndSubs();


        /// Gazebo node.
        transport::NodePtr node_handle_;
        transport::PublisherPtr imu_pub_;

        /// Pointer to the world.
        physics::WorldPtr world_;

        /// Pointer to the model.
        physics::ModelPtr model_;

        /// Pointer to the link.
        physics::LinkPtr link_;

        /// Pointer to the update event connection.
        event::ConnectionPtr updateConnection_;

        /// IMU message. This is modified everytime OnUpdate() is called, and then published onto a topic
        gz_sensor_msgs::Imu imu_message_;

        string namespace_;
        string imu_topic_;
        string frame_id_;
        string link_name_;
        common::Time last_time_;

        IMU imu;

        math::Vector3 gravity_W_;

        NoiseParameters gyro_params;
        NoiseParameters acc_params;
    };
}

#endif // IMU_PLUGIN_H