#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

# camera
sudo apt install libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev 
## librealsense
mkdir -p thirdparties/
cd thirdparties/

if [ -d "librealsense" ]; then
    echo "librealsense directory already exists. Skipping clone."
else
    echo "Cloning librealsense..."
    git clone https://github.com/yutian929/YanBot-Sense_librealsense.git librealsense
    check_success
fi

cd librealsense
./scripts/setup_udev_rules.sh
check_success

if [ -d "build" ]; then
    echo "Removing existing librealsense build directory..."
    rm -rf build
fi
mkdir build && cd build
cmake ..
check_success
make -j$(( $(nproc) / 2 ))
check_success
sudo make install
check_success
cd ../../..
## realsense-ros
mkdir -p src/camera/
cd src/camera/
if [ -d "realsense_ros" ]; then
    echo "realsense_ros directory already exists. Skipping clone."
else
    echo "Cloning realsense_ros..."
    git clone https://github.com/yutian929/YanBot-Sense_realsense_ros.git realsense_ros
    check_success
fi
cd ../../

echo "successfully build librealsense and create realsense-ros."
echo "you can run: realsense-viewer to verify the installation."
