# Tutorial on Launch files

This document uses the ros_demo/launch/mav_hovering_demo.launch as an example. 

## Launch file
The wiki for `roslaunch` can be found [here](http://wiki.ros.org/roslaunch). Some tips on using launch files for large projects are given [here](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects).

## `roslaunch` Structure
A launch file contains other launch files and all dependencies, nodes, parameters and flags that go with running a simulation of a robot. 

```xml
<include file="$(find rotors_gazebo)/launch/spawn_mav.launch"> 
<arg name="mav_name" value="$(arg mav_name)" /> 
<arg name="model" value="$(find rotors_description)/urdf/mav_generic_odometry_sensor.gazebo" /> 
<arg name="enable_logging" value="$(arg enable_logging)" /> 
<arg name="enable_ground_truth" value="$(arg enable_ground_truth)" /> 
<arg name="log_file" value="$(arg log_file)"/> 
</include> 
```xml

### Nodes
The following code includes the lee_position_controller_node ROS node from the rotors_control package. This package includes all references to the control gains and vehicle parameters that the controller needs. Three types of commands are possible: Trajectory, motor speed and pose. 

```xml
<node name="lee_position_controller_node" pkg="rotors_control" type="lee_position_controller_node" output="screen"> 
<rosparam command="load" file="$(find rotors_gazebo)/resource/lee_controller_$(arg mav_name).yaml" /> 
<rosparam command="load" file="$(find rotors_gazebo)/resource/$(arg mav_name).yaml" /> 
<remap from="odometry" to="odometry_sensor1/odometry" /> 
</node>
```

The hovering_example.cpp file from the rotors_gazebo package is used to input a trajectory command for this example. It uses a ros node that publishes to the mav_msgs::default_topics::COMMAND_TRAJECTORY topic. The message type is trajectory_msgs::MultiDOFJointTrajectory. 

```xml
<node name="hovering_example" pkg="rotors_gazebo" type="hovering_example" output="screen"/> 
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" /> 
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" /> 
```

### User Extensions 
Custom code can go at the end as follows: See mav_hovering_demo.launch 

```xml
<node name="data_logger" pkg="ros_demo" type="data_logger" /> 
<node pkg="rosbag" type="record" name="cmd" output="screen" args="-o /home/kakul/catkin_ws/bagfiles/$(arg mav_name)_cmd --topic /$(arg mav_name)/command/trajectory"/> 
```
 
The first line shows that I am compiling source code from the ros_demo package and intend to use the data_logger.cpp in the simulation 

The second line shows that I intend to record the data and create a rosbag. The arguments have the reference to the output directory. Note the path needs to be specified in full and has to exist. I.e. the catkin_ws/bagfiles directory has to exist. This line also shows that we are recording data only from /command/trajectory topic. 