#ifndef STATE_ESTIMATION_NODE_H
#define STATE_ESTIMATION_NODE_H
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <string>
#include "UKF.h"

using namespace std;
namespace ros_demo {
    class StateEstimationNode
    {
        public:
        StateEstimationNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
        ~StateEstimationNode();

        void InitializeParams();
        void Publish();
        
        private:
        string mav_name_ = "firefly";
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // LeePositionController lee_position_controller_;

        std::string namespace_;

        // subscribes to imu sensor messages from the imu sensor topic
        string imu_topic_name_ = "demo_imu";
        ros::Subscriber imu_sub_;

        // publishes to odometry topic for lee controller
        string pub_topic_name_ = "odometry_sensor2/odometry";
        ros::Publisher odometry_pub_;

        void ImuMsgCallback(const sensor_msgs::ImuConstPtr& imu_msg);

        UKF ukf_;
    };
}

#endif // STATE_ESTIMATION_NODE_H