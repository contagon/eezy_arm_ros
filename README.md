  # Introduction
This package is aimed at controlling the [EEZYbotARM MK2](http://www.eezyrobots.it/eba_mk2.html) and implementing kinematic control of the robot. Implemented algorithms include forward kinematics, inverse kinematics, and a spline-based trajectory generator. Furthered, a controller node has been implemented to smoothly send commands to the servos using the `gpiozero` library. All of this has been implemented in ROS for easy usage. 

Here's an example of a trajectory generation of a figure 8

https://user-images.githubusercontent.com/36386973/145510658-5c7a330b-1b3d-463a-999d-a5e55e9ea404.mp4

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
$ sudo apt install build-essential ros-noetic-rqt ros-noetic-rqt-graph --yes
# Create and build catkin workspace
$ mkdir -p ~/catkin_ws/src
$ cd catkin_ws/
$ catkin_make
$ source devel/setup.bash
# Clone this repo into the src directory
$ cd src/
$ git clone git@github.com:contagon/eezy_arm_ros.git
```

## Build SciPy from source
```shell
# Install necessary libraries/packages
$ sudo apt-get install gcc g++ gfortran python3-dev libopenblas-dev liblapack-dev
$ pip3 install pybind11 cython pythran
# Download and build
$ cd ~/
$ git clone git@github.com:scipy/scipy.git
$ cd scipy/
$ git checkout v1.7.0 # If using Python 3.7
# Need to init submodules (particularly Boost)
$ git submodule update --init --recursive
$ python3 setup.py install --user   # Installs to your home directory
# Hang tight, it will be a while ;)
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
There's a number of predefined trajectories you can send as well! Try either one of the following scripts
```shell
$ rosrun eezy_arm_ros run_axis.py y
$ rosrun eezy_arm_ros run_figure_eight.py
```
To speed up or slow it down, set the `max_accel` rosparam either in `start.launch` or via the command
```shell
$ rosparam set /arm1/max_accel 1000.0
```
