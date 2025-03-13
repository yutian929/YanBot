#!/bin/bash

# Public function
logfile="dep_install.log"

function echo_fail(){
    echo -e "\t[Error] $@" >> "${logfile}"
    echo -e "\e[31m[Error] $@\e[0m"
}
function echo_warn(){
    echo -e "\t[Warn] $@" >> "${logfile}"
    echo -e "\e[33m[Warn] $@\e[0m"
}
function echo_info(){
    echo -e "\t[Info] $@" >> "${logfile}"
    echo -e "\e[32m[Info] $@\e[0m"
}


# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}
# Check if the YanBot in ENV
check_ws() {
    echo "[INFO] Checking environment variables YANBOT_WS"
    if [ -z "$YANBOT_WS" ]; then
        echo_fail "Environment variable YANBOT_WS not set, Recheck your configuration"
        exit 1
    else
        echo_info "YANBOT_WS Path: $YANBOT_WS"
        cd $YANBOT_WS
    fi
}
# Check if sudo/root
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        echo_fail "Please run with sudo"
        exit 1
    fi
}

# Now Main Process

check_ws

rosdep install --from-paths src --ignore-src -r -y
check_success

sudo apt install -y libeigen3-dev
check_success

# Check if /usr/include/Eigen already exists
if [ ! -d "/usr/include/Eigen" ]; then
    echo "Copying Eigen headers to /usr/include/Eigen"
    sudo cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R
    check_success
else
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


ws_path=$YANBOT_WS
echo_info "Installing AstraSDK, SDK should be in $ws_path/src/thirdparties/AstraSDK"
cd src/thirdparties/AstraSDK/
sudo bash install/install.sh
check_success
# Assuming ASTRA_SDK is in the workspace
echo "export ASTRA_SDK_INCLUDE=$ws_path/src/thirdparties/AstraSDK/include" >> ~/.bashrc
echo "export ASTRA_SDK_LIB=$ws_path/src/thirdparties/AstraSDK/lib" >> ~/.bashrc
source ~/.bashrc
check_success
cd ../../

# Install libuvc
echo_info "Compiling libuvc"
mkdir temp
cd temp
git clone https://github.com/ktossell/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make
sudo make install
check_success
cd ../../
rm -rf temp
check_success

echo_info "All Shell Tasks Done"

# catkin_make
# check_success
