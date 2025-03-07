#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

# RTAB-MAP
sudo apt install ros-${ROS_DISTRO}-rtabmap-ros
check_success
