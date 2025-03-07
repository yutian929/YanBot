#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

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
