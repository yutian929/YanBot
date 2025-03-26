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

# Clone grounding_sam_ros
mkdir -p src/Cerebellum/
cd src/Cerebellum/

if [ -d "depth_anything_ros" ]; then
    echo "depth_anything_ros directory already exists. Skipping clone."
else
    echo "Cloning depth_anything_ros..."
    git clone https://github.com/zjy109/YanBot-depth_anything_ros.git depth_anything_ros
    check_success
fi

cd depth_anything_ros/

# Download the checkpoints
cd weights/
bash download_weights.sh
check_success
cd ../

check_ws
