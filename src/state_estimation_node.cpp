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
        // odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/firefly/odometry_sensor2/odometry", 1);
        odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(pub_topic_name_, 10);
        // Publish();
    }

    StateEstimationNode::~StateEstimationNode() { }

    void StateEstimationNode::InitializeParams()
    {
        // Read parameters from rosparam.
        rotors_control::GetRosParameter(private_nh_, "mav_name",mav_name_,&mav_name_);
        rotors_control::GetRosParameter(private_nh_, "imu_topic_name",imu_topic_name_,&imu_topic_name_);
        rotors_control::GetRosParameter(private_nh_, "pub_topic_name",pub_topic_name_,&pub_topic_name_);
        rotors_control::GetRosParameter(private_nh_, "publish_rate",publish_rate_,&publish_rate_);
        ukf_ = UKF();
        seq = 0;
    }
    void StateEstimationNode::Publish()
    {
        ros::Time time_now = ros::Time::now();
        if (is_initialized)
        {
            double dt = (time_now-prev_time).toSec();
            if (dt > 0)
            {
                total_sim_time += dt;
                ROS_INFO_ONCE("Predicting with dt = %.5f",dt);
                ukf_.Prediction(dt);

                if (measurement_received_) ukf_.UpdateIMU(imu_data_);
                nav_msgs::OdometryPtr odometry_msg(new nav_msgs::Odometry);
                odometry_msg->header.seq = seq++;
                odometry_msg->header.stamp = time_now;
                
                odometry_msg->pose.pose.orientation.w = ukf_.x_(0);//0;//
                odometry_msg->pose.pose.orientation.x = ukf_.x_(1);0;//
                odometry_msg->pose.pose.orientation.y = ukf_.x_(2);0;//
                odometry_msg->pose.pose.orientation.z = ukf_.x_(3);0;//

                odometry_msg->pose.pose.position.x = ukf_.x_(4);
                odometry_msg->pose.pose.position.y = ukf_.x_(5);
                odometry_msg->pose.pose.position.z = ukf_.x_(6);

                odometry_msg->twist.twist.linear.x = ukf_.x_(7);
                odometry_msg->twist.twist.linear.y = ukf_.x_(8);
                odometry_msg->twist.twist.linear.z = ukf_.x_(9);

                odometry_msg->twist.twist.angular.x = ukf_.x_(10);
                odometry_msg->twist.twist.angular.y = ukf_.x_(11);
                odometry_msg->twist.twist.angular.z = ukf_.x_(12);

                odometry_pub_.publish(odometry_msg);
            }
        }
        prev_time = time_now;
        if (!is_initialized) is_initialized = true;
        measurement_received_ = false;
    }

    void StateEstimationNode::ImuMsgCallback(const sensor_msgs::ImuConstPtr& imu_msg)
    {
        ROS_INFO_ONCE("mav_name_: %s", mav_name_.c_str());
        ROS_INFO_ONCE("imu_topic_name_: %s", imu_topic_name_.c_str());
        ROS_INFO_ONCE("pub_topic_name_: %s", pub_topic_name_.c_str());
        ROS_INFO_ONCE("publish_rate_: %d", publish_rate_);
        ROS_INFO_ONCE("Current Z acceleration: %.2f", imu_msg->linear_acceleration.z);
        measurement_received_=true;
        imu_data_ = imu_msg;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_estimation_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros_demo::StateEstimationNode state_estimation_node(nh,private_nh);
    // ros::Rate loop_rate(state_estimation_node.publish_rate_);

    while (ros::ok())
    {
        state_estimation_node.Publish();

        // loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
