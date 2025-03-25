#!/bin/bash


# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}
# Check if the YanBot in ENV
check_ws() {
    if [ -z "$YANBOT_WS" ]; then
        echo "YANBOT_WS is not set. Exiting."
        exit 1
    else 
        echo "YANBOT_WS is set to $YANBOT_WS"
        cd $YANBOT_WS
    fi
}

# Check if sudo/root
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        echo "Please run with sudo"
        exit 1
    fi
}

# Now Main Process

check_ws

rosdep install --from-paths src --ignore-src -r -y
check_success

sudo apt install udev
check_success

sudo apt install can-utils
check_success

sudo apt install -y libeigen3-dev
check_success

# Check if /usr/include/Eigen already exists
if [ ! -d "/usr/include/Eigen" ]; then
    echo "Copying Eigen headers to /usr/include/Eigen"
    sudo cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R
    check_success
else
    sudo cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R
    echo "/usr/include/Eigen already exists, skipping copy"
fi

# Some deps to install
sudo apt-get install libpcap-dev -y
check_success

sudo apt-get install python3-testresources -y
check_success

sudo pip3 install --upgrade rosdep
check_success

sudo apt install libcjson1 libcjson-dev
check_success

sudo apt install ros-noetic-robot-pose-ekf
check_success


mkdir -p $YANBOT_WS/thirdparties/
cd $YANBOT_WS/thirdparties/
check_success

if [ -d "AstraSDK" ]; then
    echo "AstraSDK directory already exists. Skipping clone."
else
    echo "Cloning AstraSDK..."
    git clone https://github.com/yutian929/YanBot-AstraSDK.git AstraSDK
    check_success
fi
cd AstraSDK
sudo bash install/install.sh
check_success

# Assuming ASTRA_SDK is in the workspace
echo "export ASTRA_SDK_INCLUDE=$YANBOT_WS/thirdparties/AstraSDK/include" >> ~/.bashrc
echo "export ASTRA_SDK_LIB=$YANBOT_WS/thirdparties/AstraSDK/lib" >> ~/.bashrc
source ~/.bashrc
check_success
cd ../

# Install libuvc
echo "Compiling libuvc"
if [ -d "libuvc" ]; then
    echo "libuvc directory already exists. Skipping clone."
else
    echo "Cloning libuvc..."
    git clone https://github.com/ktossell/libuvc libuvc
    check_success
fi
cd libuvc
if [ -d "build" ]; then
    echo "Removing existing librealsense build directory..."
    rm -rf build
fi
mkdir build
cd build
cmake ..
make
sudo make install
check_success

echo "All Shell Tasks Done"
