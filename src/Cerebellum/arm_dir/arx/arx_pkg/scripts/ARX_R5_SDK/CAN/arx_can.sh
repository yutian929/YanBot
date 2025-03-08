#!/bin/bash

# Source the configuration file
source ~/.bashrc

CAN_DEVICE=$1  # Device path, e.g., /dev/arxcan0
CAN_INTERFACE=$2  # Network interface, e.g., can0

start_can() {
    echo "Starting slcand..."
    sudo slcand -o -f -s8 $CAN_DEVICE $CAN_INTERFACE
    if [ $? -ne 0 ]; then
        echo "Failed to start slcand"
        return 1
    fi
    echo "Configuring $CAN_INTERFACE interface..."
    sudo ifconfig $CAN_INTERFACE up
    if [ $? -ne 0 ]; then
        echo "Failed to bring up $CAN_INTERFACE interface: RTNETLINK answers: Operation not supported"
        return 1
    fi
    echo "$CAN_INTERFACE started successfully"
    return 0
}

check_can() {
    if ip link show "$CAN_INTERFACE" > /dev/null 2>&1; then
        if ip link show "$CAN_INTERFACE" | grep -q "UP"; then
            return 0
        else
            return 1
        fi
    else
        return 2
    fi
}

while true; do
    if check_can; then
        echo "CAN interface $CAN_INTERFACE is working properly"
    else
        echo "$CAN_INTERFACE is down, restarting..."
        sudo ip link set $CAN_INTERFACE down
        sudo pkill -9 slcand
        sleep 1

        if ! start_can; then
            echo "Failed to restart CAN interface, please check hardware or driver."
        fi
    fi
done
