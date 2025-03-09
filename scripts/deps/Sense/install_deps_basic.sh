#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

# check whether $YANBOT_WS existed.
check_ws() {
    if [ -z "$YANBOT_WS" ]; then
        echo "YANBOT_WS is not set. Exiting."
        exit 1
    else 
        echo "YANBOT_WS is set to $YANBOT_WS"
        cd $YANBOT_WS
    fi
}

# check and goto YANBOT_WS
check_ws

# basic
## common
sudo apt update
check_success
# sudo apt upgrade
# check_success
sudo apt install python3-pip git
check_success
sudo apt install ros-${ROS_DISTRO}-image-transport
check_success
sudo apt install ros-${ROS_DISTRO}-rgbd-launch
check_success
sudo apt install ros-${ROS_DISTRO}-ddynamic-reconfigure
check_success
sudo apt-get install ros-$ROS_DISTRO-octomap-server
check_success
sudo apt install v4l-utils
check_success
