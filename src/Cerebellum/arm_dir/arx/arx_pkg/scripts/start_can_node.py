#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Start CAN Communication Node for ARX Dual-Arm Desktop Robot

Author: yutian
Date: 2024-12-17
"""

import rospy
import subprocess
import time
import signal
import os
import sys

# Define different CAN devices and their corresponding interfaces
CAN_DEVICES = [
    ("/dev/arxcan0", "can0"),
    ("/dev/arxcan1", "can1"),
    ("/dev/arxcan2", "can2"),
    ("/dev/arxcan3", "can3"),
]

# Path to the CAN initialization script
CAN_BASH_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "ARX_R5_SDK", "CAN", "arx_can.sh"
)

# List to store subprocesses for each CAN interface
processes = []


def welcome():
    """
    Display a welcome message to the user.
    """
    rospy.loginfo("* Node: arx_pkg/start_can_node.py")
    rospy.loginfo("* AUTHOR: yutian")
    rospy.loginfo(
        "* NOTES: 2024-12-17 v0 || This script aim to start CAN communication between PC and ARX Dual-Arm Desktop Robot."
    )


def signal_handler(sig, frame):
    """
    Handle SIGINT (Ctrl+C) to gracefully shutdown the node.
    """
    rospy.loginfo("Shutdown signal received. Terminating all CAN devices...")
    for process in processes:
        rospy.loginfo(f"Terminating process with PID {process.pid}...")
        process.terminate()  # Gracefully terminate the process
        try:
            process.wait(timeout=5)  # Wait for the process to terminate
            rospy.loginfo(f"Process {process.pid} terminated successfully.")
        except subprocess.TimeoutExpired:
            rospy.logwarn(
                f"Process {process.pid} did not terminate in time. Killing it..."
            )
            process.kill()  # Forcefully kill the process if it doesn't terminate
            rospy.loginfo(f"Process {process.pid} killed.")
    sys.exit(0)


def start_can_device(device, interface):
    """
    Start a single CAN interface by invoking the arx_can.sh script.

    Args:
        device (str): The CAN device path (e.g., /dev/arxcan0).
        interface (str): The CAN interface name (e.g., can0).

    Returns:
        subprocess.Popen or None: The subprocess object if started successfully, else None.
    """
    try:
        rospy.loginfo(f"Starting CAN device {device} with interface {interface}...")
        # Use subprocess.Popen to start the script in a non-blocking way
        process = subprocess.Popen([CAN_BASH_FILE, device, interface])
        return process
    except Exception as e:
        rospy.logerr(f"Failed to start interface {interface}: {e}")
        return None


def start_can_node():
    """
    Main function to start all CAN interfaces and keep the node running.
    """
    global processes  # Declare processes as a global variable

    rospy.init_node("setup_can_node", anonymous=True)
    rospy.loginfo("CAN Setup Node Initialized.")

    # Register the signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    welcome()

    # Verify that the CAN initialization script exists and is executable
    if not os.path.isfile(CAN_BASH_FILE):
        rospy.logerr(f"CAN initialization script not found: {CAN_BASH_FILE}")
        sys.exit(1)
    if not os.access(CAN_BASH_FILE, os.X_OK):
        rospy.logerr(f"CAN initialization script is not executable: {CAN_BASH_FILE}")
        sys.exit(1)

    # Start each CAN device
    for device, interface in CAN_DEVICES:
        process = start_can_device(device, interface)
        if process:
            processes.append(process)  # Add the process to the list
        else:
            rospy.logerr(f"Failed to start CAN interface {interface}.")
        time.sleep(1)  # Wait a bit to avoid rapid subprocess spawning

    rospy.loginfo("All CAN interfaces have been started. Node is now running.")

    # Keep the node alive until shutdown is triggered
    rospy.spin()


if __name__ == "__main__":
    try:
        start_can_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception caught. Exiting setup_can_node.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        sys.exit(1)
