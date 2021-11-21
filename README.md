This was all built on a raspberry pi. This was all done on 64bit Buster image, found [here](https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2021-05-28/https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2021-05-28/)

# Install ROS
* `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
* `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
* `sudo apt update`
* `sudo apt install ros-noetic-ros-core`
* `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`

# Enable GPIO libraries
* `sudo systemctl enable pigpiod.service`

# Setup ROS
* `mkdir ~/catkin_ws/src`
* `cd catkin_ws`
* `sudo apt install build-essential`
* `catkin_make`
* `source devel/setup.bash`
* Clone this repo into the src directory
