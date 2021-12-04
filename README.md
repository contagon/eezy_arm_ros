# Introduction
This package is aimed at controlling the [EEZYbotARM MK2](http://www.eezyrobots.it/eba_mk2.html) and implementing kinematic control of the robot. Initial goals are to implement forward kinematics, and inverse kinematics.

This was all built on a raspberry pi running 64-bit Buster. Image can be found [here](https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2021-05-28/).

# Initial Setup
## Install ROS
```shell
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt install ros-noetic-ros-core
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Enable GPIO libraries
```shell
$ sudo systemctl enable pigpiod.service
```

## Setup ROS
```shell
# Install necessary packages (if not already installed)
$ sudo apt install build-essential
# Create and build catkin workspace
$ mkdir -p ~/catkin_ws/src
$ cd catkin_ws/
$ catkin_make
$ source devel/setup.bash
# Clone this repo into the src directory
$ cd src/
$ git clone git@github.com:contagon/eezy_arm_ros.git
```

# Usage
After running `catkin_make` and `source devel/setup.bash`, to launch all nodes, run
```shell
$ roslaunch eezy_arm_ros start.launch
```
Then, to send a desired position or joint angle, run one of the following commands
```shell
$ rostopic pub -1 /arm1/p_des geometry_msgs/Vector3 -- 200 -100 250
$ rostopic pub -1 /arm1/q_des eezy_arm_ros/Joints -- 0 0 0
```