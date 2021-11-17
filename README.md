This was all built on a raspberry pi. I started with Ubuntu Server 20.04 to ease the load of installing ROS (currently Raspbian is on bullseye, which no version of ROS officially supports...). I set up dependencies as follows:

# Install ROS
* `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
* `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
* `sudo apt update`
* `sudo apt install ros-noetic-ros-core`
* `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`

# Install GPIO libraries
* `sudo apt-get install python3-pigpio python3-gpiozero`
* Install pigpio from source, [instructions here](https://abyz.me.uk/rpi/pigpio/download.html)

# Setup libraries
* `mkdir ~/catkin_ws/src`
* `cd catkin_ws`
* `sudo apt install build-essential`
* `catkin_make`
* `source devel/setup.bash`
* clone this repo into the src directory
