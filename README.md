# ros_demo
ROS/Gazebo demo using the ROTORS framework
Rotors Github page can be accessed [here](https://github.com/ethz-asl/rotors_simulator). 

## Dependencies: Verified to work 
1. Ubuntu 16.04 
2. ROS Kinetic 

## Installation 

The following are the steps needed to get this repo to work with the above dependencies. Refer to the [ROTORS project page on Github](https://github.com/ethz-asl/rotors_simulator) for more details.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' 
wget http://packages.ros.org/ros.key -O - | sudo apt-key add - 
sudo apt-get update 
sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox 
sudo rosdep init 
rosdep update 
source /opt/ros/kinetic/setup.bash 
```

If you have not created a catkin workspace, create one: 
```bash
mkdir ~/catkin_ws 
mkdir ~/catkin_ws/src
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Clone the ROTORS starter code from github.
```bash
cd ~/catkin_ws/src 
git clone https://github.com/ethz-asl/rotors_simulator.git 
git clone https://github.com/ethz-asl/mav_comm.git 
cd ~/catkin_ws 
catkin build
source ~/catkin_ws/devel/setup.bash
```

### Potential Issues
There might be some issues with mav link. Install those dependencies. I recall I installed something, but don't remember what. In any case, this is only necessary if the following test fails.

## Test Installation 
```bash
roslaunch rotors_gazebo mav_hovering_example.launch
```
This will show a mav take-off after 5 seconds and hover at a height of z = 1.  

If this succeeds, all the dependencies needed to run this project have been installed correctly.

## Download project code and miscellaneous dependencies
```bash
cd ~/catkin_ws/src
git clone https://github.com/chandrusuresh/ros_demo.git
```

Clone header file that samples from a multivariate gaussian. Compatible with EIGEN.
Note: `eigenmvn.h` is already included in this repo. The following install step is included for the sake of completeness and can be skipped.
```bash
cd ~/Downloads
git clone https://github.com/beniz/eigenmvn
cp eigenmvn/eigenmvn.h ~/catkin_ws/src/ros_demo/include
```

## Run the project
```bash
cd ..
catkin build
source ~/catkin_ws/devel/setup.bash
roslaunch rotors_gazebo mav_hovering_demo.launch
```
This is essentially an identical example as above that's cloned into this repo. You are all set if this simulation runs without errors.

 
