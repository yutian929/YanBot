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

sudo apt-get update
sudo apt-get upgrade

sudo apt-get install swig libatlas-base-dev libasound2-dev portaudio19-dev libportaudio2 libportaudiocpp0 python3-pyaudio sox
check_success
pip3 install --upgrade pip
check_success
pip install pyaudio
check_success
pip install scipy
check_success
