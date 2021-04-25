#include "imu_plugin.h"

// SYSTEM LIBS
#include <stdio.h>
#include <boost/bind.hpp>
#include <chrono>
#include <cmath>
#include <iostream>

// 3RD PARTY
#include "mav_msgs/default_topics.h"

// USER HEADERS
#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {
    IMUPlugin::IMUPlugin()
    : ModelPlugin(),
      node_handle_(0) {}

    IMUPlugin::~IMUPlugin() {}

    void IMUPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        model_ = _model;
        world_ = model_->GetWorld();

        // default params
        namespace_.clear();

        //==============================================//
        //========== READ IN PARAMS FROM SDF ===========//
        //==============================================//

        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
            gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";

        // Get node handle
        node_handle_ = transport::NodePtr(new transport::Node());

        // Initialise with default namespace (typically /gazebo/default/)
        node_handle_->Init();

        if (_sdf->HasElement("linkName"))
            link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
        else
            gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
        
        // Get the pointer to the link
        link_ = model_->GetLink(link_name_);
        if (link_ == NULL)
            gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_
                                                                        << "\".");

        frame_id_ = link_name_;

        getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_,
                                mav_msgs::default_topics::IMU);
        getSdfParam<double>(_sdf, "gyroscopeNoiseDensity",
                            gyro_params.white_noise_density,
                            gyro_params.white_noise_density);
        getSdfParam<double>(_sdf, "gyroscopeBiasRandomWalk",
                            gyro_params.random_walk_density,
                            gyro_params.random_walk_density);
        getSdfParam<double>(_sdf, "gyroscopeBiasCorrelationTime",
                            gyro_params.bias_correlation_time,
                            gyro_params.bias_correlation_time);
        assert(gyro_params.bias_correlation_time > 0.0);
        getSdfParam<double>(_sdf, "gyroscopeTurnOnBiasSigma",
                            gyro_params.turn_on_bias_density,
                            gyro_params.turn_on_bias_density);
        getSdfParam<double>(_sdf, "accelerometerNoiseDensity",
                            acc_params.white_noise_density,
                            acc_params.white_noise_density);
        getSdfParam<double>(_sdf, "accelerometerRandomWalk",
                            acc_params.random_walk_density,
                            acc_params.random_walk_density);
        getSdfParam<double>(_sdf, "accelerometerBiasCorrelationTime",
                            acc_params.bias_correlation_time,
                            acc_params.bias_correlation_time);
        assert(acc_params.bias_correlation_time > 0.0);
        getSdfParam<double>(_sdf, "accelerometerTurnOnBiasSigma",
                            acc_params.turn_on_bias_density,
                            acc_params.turn_on_bias_density);

        vector<NoiseParameters> gyro_params_vect(3,gyro_params);
        vector<NoiseParameters> acc_params_vect(3,acc_params);
        Sensor gyro(gyro_params_vect);
        Sensor acc(acc_params_vect);
        IMU imu(gyro,acc);

        last_time_ = world_->GetSimTime();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&IMUPlugin::OnUpdate, this, _1));

        //==============================================//
        //====== POPULATE STATIC PARTS OF IMU MSG ======//
        //==============================================//

        //  imu_message_.header.frame_id = frame_id_;
        imu_message_.mutable_header()->set_frame_id(frame_id_);

        // We assume uncorrelated noise on the 3 channels -> only set diagonal
        // elements. Only the broadband noise component is considered, specified as a
        // continuous-time density (two-sided spectrum); not the true covariance of
        // the measurements.

        for (int i = 0; i < 9; i++) {
            switch (i) {
            case 0:
                imu_message_.add_angular_velocity_covariance(
                    pow(gyro_params.white_noise_density,2));

                imu_message_.add_orientation_covariance(-1.0);

                imu_message_.add_linear_acceleration_covariance(
                    pow(acc_params.white_noise_density,2));
                break;
            case 1:
            case 2:
            case 3:
                imu_message_.add_angular_velocity_covariance(0.0);

                imu_message_.add_orientation_covariance(-1.0);

                imu_message_.add_linear_acceleration_covariance(0.0);
                break;
            case 4:
                imu_message_.add_angular_velocity_covariance(
                    pow(gyro_params.white_noise_density,2));

                imu_message_.add_orientation_covariance(-1.0);

                imu_message_.add_linear_acceleration_covariance(
                    pow(acc_params.white_noise_density,2));
                break;
            case 5:
            case 6:
            case 7:
                imu_message_.add_angular_velocity_covariance(0.0);

                imu_message_.add_orientation_covariance(-1.0);

                imu_message_.add_linear_acceleration_covariance(0.0);
                break;
            case 8:
                imu_message_.add_angular_velocity_covariance(
                    pow(gyro_params.white_noise_density,2));

                imu_message_.add_orientation_covariance(-1.0);

                imu_message_.add_linear_acceleration_covariance(
                    pow(acc_params.white_noise_density,2));
                break;
            }
        }

        gravity_W_ = world_->GetPhysicsEngine()->GetGravity();
        // imu_parameters_.gravity_magnitude = gravity_W_.GetLength();
    }
    void IMUPlugin::OnUpdate(const common::UpdateInfo& _info)
    {
        if (isInit)
        {
            CreatePubsAndSubs();
            isInit = false;
        }

        common::Time current_time = world_->GetSimTime();
        double dt = (current_time - last_time_).Double();
        last_time_ = current_time;
        double t = current_time.Double();

        math::Pose T_W_I = link_->GetWorldPose();
        math::Quaternion C_W_I = T_W_I.rot;

        math::Vector3 acceleration_I =
            link_->GetRelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_);

        math::Vector3 angular_vel_I = link_->GetRelativeAngularVel();

        VectorXd gyro_true(3);
        gyro_true << angular_vel_I.x, angular_vel_I.y, angular_vel_I.z;

        VectorXd acc_true(3);
        acc_true << acceleration_I.x, acceleration_I.y, acceleration_I.z;
        
        VectorXd gyro_wNoise = imu.getGyroscopeReading(gyro_true,dt);
        VectorXd acc_wNoise  = imu.getAccelerometerReading(acc_true,dt);

        // Fill IMU message.
        //  imu_message_.header.stamp.sec = current_time.sec;
        imu_message_.mutable_header()->mutable_stamp()->set_sec(current_time.sec);

        //  imu_message_.header.stamp.nsec = current_time.nsec;
        imu_message_.mutable_header()->mutable_stamp()->set_nsec(current_time.nsec);


        gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
        orientation->set_w(C_W_I.w);
        orientation->set_x(C_W_I.x);
        orientation->set_y(C_W_I.y);
        orientation->set_z(C_W_I.z);
        imu_message_.set_allocated_orientation(orientation);

        gazebo::msgs::Vector3d* linear_acceleration = new gazebo::msgs::Vector3d();
        linear_acceleration->set_x(acc_wNoise[0]);
        linear_acceleration->set_y(acc_wNoise[1]);
        linear_acceleration->set_z(acc_wNoise[2]);
        imu_message_.set_allocated_linear_acceleration(linear_acceleration);

        gazebo::msgs::Vector3d* angular_velocity = new gazebo::msgs::Vector3d();
        angular_velocity->set_x(gyro_wNoise[0]);
        angular_velocity->set_y(gyro_wNoise[1]);
        angular_velocity->set_z(gyro_wNoise[2]);
        imu_message_.set_allocated_angular_velocity(angular_velocity);

        // Publish the IMU message
        imu_pub_->Publish(imu_message_);
    }
    void IMUPlugin::CreatePubsAndSubs()
    {
        // Create temporary "ConnectGazeboToRosTopic" publisher and message
        gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
            node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
                "~/" + kConnectGazeboToRosSubtopic, 1);

        // ============================================ //
        // =============== IMU MSG SETUP ============== //
        // ============================================ //

        imu_pub_ = node_handle_->Advertise<gz_sensor_msgs::Imu>(
            "~/" + namespace_ + "/" + imu_topic_, 1);

        gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
        // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
        connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                        imu_topic_);
        connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + imu_topic_);
        connect_gazebo_to_ros_topic_msg.set_msgtype(
            gz_std_msgs::ConnectGazeboToRosTopic::IMU);
        connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                                true);
    }

    GZ_REGISTER_MODEL_PLUGIN(IMUPlugin);
}