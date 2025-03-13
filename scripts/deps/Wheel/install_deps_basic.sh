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
check_workingspace() {
    echo "[INFO] 检查工作空间变量是否配置"
    if [ -z "$YANBOT_WS" ]; then
        echo_warn "没有找到关于YanBot的工作空间,请先配置环境变量"
        exit 1
    else
        echo_info "找到YanBot工作空间: $YANBOT_WS"
        cd $YANBOT_WS
    fi
}
# Check if sudo/root
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        echo_fail "请使用sudo或root权限运行此脚本"
        exit 1
    fi
}

# Now Main Process




check_workingspace

rosdep install --from-paths src --ignore-src -r -y
check_success

sudo apt install -y git
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

sudo apt-get install libpcap-dev -y
check_success

sudo apt-get install python3-testresources -y
check_success

sudo pip3 install --upgrade rosdep
check_success

sudo apt install libcjson1 libcjson-dev
check_success

echo "如果要编译bodyreader,需要下载ASTRA SDK,并且安装,然后将2个lib添加至bashrc变量,具体见/bodyreader/CMakeLists.txt"
cd src/thirdparties/AstraSDK/
sudo bash install/install.sh
check_success
echo "export ASTRA_SDK_INCLUDE=$SDK_PATH/include" >> ~/.bashrc
echo "export ASTRA_SDK_LIB=$SDK_PATH/lib" >> ~/.bashrc
check_success
source ~/.bashrc
check_success
cd ../../

echo_info "安装catkin_tools用于工作空间编译"
sudo apt install python3-catkin-tools -y
check_success

echo_info "安装rosdep"
sudo apt install python3-rosdep -y
check_success


# catkin_make --pkg lslidar_msgs lslidar_driver lslidar # 先编译lslidar的包
# check_success

echo_info "安装libuvc"
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

# Try to Compile
echo_info "依赖安装完成，尝试编译工作空间"
cd $YANBOT_WS
rm -rf build devel .catkin_tools

ws_path=$YANBOT_WS
export ASTRA_SDK_INCLUDE=$ws_path/src/thirdparties/AstraSDK/include
export ASTRA_SDK_LIB=$ws_path/src/thirdparties/AstraSDK/lib
catkin build
check_success

echo_info "shell任务完成"

# catkin_make
# check_success
