#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

// #include "lee_position_controller_node.h"
#include "state_estimation_node.h"

#include "rotors_control/parameters_ros.h"

namespace ros_demo
{
    StateEstimationNode::StateEstimationNode(
        const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):nh_(nh),private_nh_(private_nh)
    {
        InitializeParams();
        imu_sub_ = nh_.subscribe(imu_topic_name_, 1,&StateEstimationNode::ImuMsgCallback, this);
        odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/firefly/odometry_sensor2/odometry", 1);
        // odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("pub_topic_name_", 1);
    }
    
    StateEstimationNode::~StateEstimationNode() { }

    void StateEstimationNode::InitializeParams()
    {
        // Read parameters from rosparam.
        rotors_control::GetRosParameter(private_nh_, "mav_name",mav_name_,&mav_name_);
        rotors_control::GetRosParameter(private_nh_, "imu_topic_name",imu_topic_name_,&imu_topic_name_);
        rotors_control::GetRosParameter(private_nh_, "pub_topic_name",pub_topic_name_,&pub_topic_name_);
        ukf_ = UKF();
    }
    void StateEstimationNode::Publish() {}

    void StateEstimationNode::ImuMsgCallback(const sensor_msgs::ImuConstPtr& imu_msg)
    {
        ROS_INFO_ONCE("mav_name_: %s", mav_name_.c_str());
        ROS_INFO_ONCE("imu_topic_name_: %s", imu_topic_name_.c_str());
        ROS_INFO_ONCE("pub_topic_name_: %s", pub_topic_name_.c_str());
        ROS_INFO("Current Z acceleration: %.2f", imu_msg->linear_acceleration.z);
        // // Clear all pending commands.
        // command_timer_.stop();
        // commands_.clear();
        // command_waiting_times_.clear();

        // mav_msgs::EigenTrajectoryPoint eigen_reference;
        // mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
        // commands_.push_front(eigen_reference);

        // lee_position_controller_.SetTrajectoryPoint(commands_.front());
        // commands_.pop_front();
    }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_estimation_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros_demo::StateEstimationNode state_estimation_node(nh,private_nh);
  ros::spin();

  return 0;
}
