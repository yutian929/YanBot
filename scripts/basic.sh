#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

# get the current working directory (run this script from the root of the workspace)
WS_DIR=$(pwd)
# export YANBOT_WS=WS_DIR to bashrc
echo "export YANBOT_WS=$WS_DIR" >> ~/.bashrc
source ~/.bashrc
cd $YANBOT_WS
