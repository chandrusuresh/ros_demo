#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <vector>

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  ROS_INFO("Current Z position: %.2f", msg->position.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_logger");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/firefly/odometry_sensor1/pose", 1000, poseCallback);
  ROS_INFO("Inside logger");

  ros::spin();

  return 0;
}