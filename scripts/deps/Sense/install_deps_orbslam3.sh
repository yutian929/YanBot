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

# Install required dependencies
sudo apt install -y libeigen3-dev
check_success
# sudo cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R
# check_success

# Optional: Install Hector Trajectory Server
sudo apt install -y ros-${ROS_DISTRO}-hector-trajectory-server
check_success

# Build Pangolin
mkdir -p thirdparties/
cd thirdparties/

if [ -d "Pangolin" ]; then
    echo "Pangolin directory already exists. Skipping clone."
else
    echo "Cloning Pangolin..."
    git clone https://github.com/yutian929/YanBot-Sense_Pangolin.git Pangolin
    check_success
fi

# Remove build directory if it exists, then build Pangolin
cd Pangolin
if [ -d "build" ]; then
    echo "Removing existing Pangolin build directory..."
    rm -rf build
fi
mkdir build && cd build
cmake ..
check_success
make -j$(( $(nproc) / 2 ))
check_success
sudo make install
check_success
sudo ldconfig  # 刷新系统库链接
check_success
cd ../../..

# Clone orb_slam3_ros
mkdir -p src/Cerebellum/
cd src/Cerebellum/

if [ -d "orb_slam3_ros" ]; then
    echo "orb_slam3_ros directory already exists. Skipping clone."
else
    echo "Cloning orb_slam3_ros..."
    git clone https://github.com/yutian929/YanBot-Sense_orb_slam3_ros.git orb_slam3_ros
    check_success
fi
cd ../..

echo "Dependencies installed successfully."
