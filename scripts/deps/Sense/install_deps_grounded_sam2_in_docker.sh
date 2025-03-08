#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

apt-get update && apt-get upgrade
check_success

apt-get install -y sudo
check_success

# echo "deb [trusted=yes arch=amd64] http://deb.repo.autolabor.com.cn jammy main" | sudo tee /etc/apt/sources.list.d/autolabor.list
# check_success

# cd /

# sudo apt-get update
# sudo apt-get install ros-noetic-autolabor
# check_success

# pip3 install catkin_pkg
# check_success
# pip3 install rospkg
# check_success


echo "Installation completed successfully!"
cd /home/appuser/Grounded-SAM-2/
