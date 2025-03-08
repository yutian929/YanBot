#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Setup CAN Communication Node for ARX Dual-Arm Desktop Robot

Author: yutian
Date: 2024-12-17
"""

import rospy
import re
import os
import time
import subprocess
import signal
import sys

# CAN rules file name
CAN_RULES_FILE = "arx_can.rules"

# Default CAN to robotic arm mapping
DEFAULT_CORRESPONDENCE = {
    "arxcan0": "Left Hand - Teach Pendant",
    "arxcan1": "Left Hand - Actuator",
    "arxcan2": "Right Hand - Teach Pendant",
    "arxcan3": "Right Hand - Actuator",
}


def signal_handler(sig, frame):
    """
    Handle SIGINT (Ctrl+C) to gracefully shutdown the node.
    """
    rospy.loginfo("Shutdown signal received. Exiting setup_can_node.")
    sys.exit(0)


def ensure_devices_clean():
    """
    Prompt the user to confirm that all CAN-related devices have been disconnected.

    Returns:
        bool: True if devices are clean, False otherwise.
    """
    rospy.loginfo("Have all CAN-related devices been disconnected? (y/n)")
    user_input = input()
    if user_input.lower() == "y":
        rospy.loginfo("Starting CAN pairing process...")
        return True
    else:
        rospy.loginfo("Please disconnect all CAN-related devices and rerun the script.")
        return False


def ensure_update_system():
    """
    Prompt the user to confirm if the system rules file needs to be updated.

    Returns:
        bool: True if the user agrees to update, False otherwise.
    """
    rospy.loginfo("Do you want to update the system rules file? (y/n)")
    user_input = input()
    if user_input.lower() == "y":
        rospy.loginfo("Updating system rules file...")
        return True
    else:
        rospy.loginfo("User canceled the system rules file update. Exiting program.")
        return False


def get_all_tty_devices():
    """
    Retrieve all /dev/tty* devices currently connected to the system.

    Returns:
        set: A set of full paths to tty devices.
    """
    tty_devices = set()
    try:
        dev_dir = "/dev"
        for device in os.listdir(dev_dir):
            if device.startswith("tty"):
                tty_devices.add(os.path.join(dev_dir, device))
    except Exception as e:
        rospy.logerr(f"Error while retrieving tty devices: {e}")

    return tty_devices


def get_can_device_info(device):
    """
    Execute udevadm command to query information about a CAN device.

    Args:
        device (str): The device path (e.g., /dev/ttyUSB0).

    Returns:
        str or None: The udevadm output if successful, None otherwise.
    """
    try:
        result = subprocess.run(
            f"udevadm info --query=all --name={device}",
            shell=True,
            text=True,
            capture_output=True,
        )

        if result.returncode == 0:
            return result.stdout
        else:
            return None
    except Exception as e:
        rospy.logerr(f"Error while searching for CAN devices: {e}")
        return None


def parse_can_device_info(result):
    """
    Parse udevadm output to extract device information such as vendor ID, product ID, and serial number.

    Args:
        result (str): The output from udevadm command.

    Returns:
        dict: A dictionary containing idVendor, idProduct, and serial.

    Raises:
        ValueError: If parsing fails.
    """
    rospy.logdebug(f"Parsing CAN device info: {result}")

    vendor_id_pattern = re.compile(r"ID_VENDOR_ID=([0-9a-fA-F]+)")
    product_id_pattern = re.compile(r"ID_MODEL_ID=([0-9a-fA-F]+)")
    serial_pattern = re.compile(r"ID_SERIAL_SHORT=([^\n]+)")
    # breakpoint()
    id_vendor = re.search(vendor_id_pattern, result)
    id_product = re.search(product_id_pattern, result)
    serial = re.search(serial_pattern, result)

    if all([id_vendor, id_product, serial]):
        return {
            "idVendor": id_vendor.group(1),
            "idProduct": id_product.group(1),
            "serial": serial.group(1),
        }
    else:
        raise ValueError("Failed to parse CAN device information")


def find_new_can_device(initial_ttys, existing_can_ttys):
    """
    Continuously search for a new CAN device that wasn't present initially or already identified.

    Args:
        initial_ttys (set): The set of tty devices present at the start.
        existing_can_ttys (set): The set of already identified CAN tty devices.

    Returns:
        str: The path to the newly detected CAN device.

    Exits:
        If multiple new CAN devices are detected simultaneously.
    """
    while not rospy.is_shutdown():
        all_ttys = get_all_tty_devices()
        new_ttys = all_ttys - initial_ttys - existing_can_ttys
        if new_ttys:
            if len(new_ttys) == 1:
                new_device = tuple(new_ttys)[0]
                rospy.loginfo(f"Detected new CAN device: {new_device}")
                return new_device
            else:
                rospy.logerr(f"Multiple new CAN devices detected: {new_ttys}. Exiting.")
                sys.exit(1)
        time.sleep(1)  # Avoid busy waiting


def welcome():
    """
    Display a welcome message to the user.
    """
    rospy.loginfo("* Node: arx_pkg/setup_can_node.py")
    rospy.loginfo("* AUTHOR: yutian")
    rospy.loginfo(
        "* NOTES: 2024-12-17 v1 || This script needs to be run only once. Please ensure all CAN devices are disconnected before proceeding."
    )


def get_can_devices(initial_ttys):
    """
    Detect and map all CAN devices based on the default correspondence.

    Args:
        initial_ttys (set): The set of tty devices present at the start.

    Returns:
        dict: A dictionary mapping device names to their information.
    """
    can_devices = {}
    identified_can_ttys = set()

    for device, description in DEFAULT_CORRESPONDENCE.items():
        rospy.loginfo(
            f"Setting up `{device}` (Default: {description}). Please connect the corresponding CAN2USB device. Do not disconnect previously connected devices."
        )
        new_can_tty = find_new_can_device(initial_ttys, identified_can_ttys)
        time.sleep(1)
        device_info_raw = get_can_device_info(new_can_tty)
        if device_info_raw:
            try:
                can_devices[device] = parse_can_device_info(device_info_raw)
                identified_can_ttys.add(new_can_tty)
            except ValueError as ve:
                rospy.logerr(f"Error parsing device info for {new_can_tty}: {ve}")
                sys.exit(1)
        else:
            rospy.logerr(f"Failed to retrieve device info for {new_can_tty}.")
            sys.exit(1)

    rospy.loginfo("\nAll CAN devices detected. Device information:")
    for device, info in can_devices.items():
        rospy.loginfo(f"{device}: {info}")

    return can_devices


def update_can_rules(can_devices, rules_file=CAN_RULES_FILE):
    """
    Generate and write udev rules based on detected CAN devices.

    Args:
        can_devices (dict): The dictionary containing CAN device information.
        rules_file (str): The filename for the udev rules.
    """
    new_rules = []
    for device, info in can_devices.items():
        rule = (
            f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{info["idVendor"]}", '
            f'ATTRS{{idProduct}}=="{info["idProduct"]}", '
            f'ATTRS{{serial}}=="{info["serial"]}", SYMLINK+="{device}"\n'
        )
        new_rules.append(rule)

    try:
        with open(rules_file, "w", encoding="utf-8") as f:
            f.writelines(new_rules)
        rospy.loginfo(f"Successfully updated rules file: {rules_file}")
    except Exception as e:
        rospy.logerr(f"Failed to write rules file {rules_file}: {e}")
        sys.exit(1)


def update_system_rules(rules_file=CAN_RULES_FILE):
    """
    Copy the rules file to the system udev rules directory and reload the rules.

    Args:
        rules_file (str): The filename of the udev rules.
    """
    try:
        dest_path = f"/etc/udev/rules.d/{rules_file}"
        rospy.loginfo(f"Copying rules file {rules_file} to {dest_path}...")

        subprocess.run(["sudo", "cp", rules_file, dest_path], check=True)
        time.sleep(1)

        rospy.loginfo(f"Setting executable permissions for {dest_path}...")
        subprocess.run(["sudo", "chmod", "755", dest_path], check=True)
        time.sleep(1)

        rospy.loginfo("Reloading udev rules and triggering changes...")
        subprocess.run(["sudo", "udevadm", "control", "--reload-rules"], check=True)
        subprocess.run(["sudo", "udevadm", "trigger"], check=True)
        time.sleep(1)

        rospy.loginfo("Rules file successfully updated and applied!")
    except subprocess.CalledProcessError as cpe:
        rospy.logerr(f"System update failed: {cpe}")
        sys.exit(1)
    except Exception as e:
        rospy.logerr(f"Error updating system rules: {e}")
        sys.exit(1)


def setup_can_node():
    """
    Initialize the ROS node and handle the setup process for CAN communication.
    """
    rospy.init_node("setup_can_node", anonymous=True)

    # Register the signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    welcome()

    if not ensure_devices_clean():
        sys.exit(0)

    initial_ttys = get_all_tty_devices()

    # Detect CAN devices
    can_devices = get_can_devices(initial_ttys)
    # can_devices = {
    #     "arxcan0": {"idVendor": "1a86", "idProduct": "7523", "serial": "0000000000000001"},
    #     "arxcan1": {"idVendor": "1a86", "idProduct": "7523", "serial": "0000000000000002"},
    #     "arxcan2": {"idVendor": "1a86", "idProduct": "7523", "serial": "0000000000000003"},
    #     "arxcan3": {"idVendor": "1a86", "idProduct": "7523", "serial": "0000000000000004"}
    # }

    # Update udev rules based on detected CAN devices
    update_can_rules(can_devices, CAN_RULES_FILE)

    if not ensure_update_system():
        sys.exit(0)

    # Apply the new udev rules to the system
    update_system_rules(CAN_RULES_FILE)

    rospy.loginfo("CAN setup process completed successfully.")


if __name__ == "__main__":
    try:
        setup_can_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception caught. Exiting setup_can_node.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        sys.exit(1)


"P: /devices/pci0000:00/0000:00:14.0/usb1/1-9/1-9.2/1-9.2:1.0/tty/ttyACM0\nN: ttyACM0\nL: 0\nS: serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2068326F5052-if00\nS: serial/by-path/pci-0000:00:14.0-usb-0:9.2:1.0\nE: DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-9/1-9.2/1-9.2:1.0/tty/ttyACM0\nE: DEVNAME=/dev/ttyACM0\nE: MAJOR=166\nE: MINOR=0\nE: SUBSYSTEM=tty\nE: USEC_INITIALIZED=1202633339\nE: ID_BUS=usb\nE: ID_VENDOR_ID=16d0\nE: ID_MODEL_ID=117e\nE: ID_PCI_CLASS_FROM_DATABASE=Serial bus controller\nE: ID_PCI_SUBCLASS_FROM_DATABASE=USB controller\nE: ID_PCI_INTERFACE_FROM_DATABASE=XHCI\nE: ID_VENDOR_FROM_DATABASE=MCS\nE: ID_VENDOR=Openlight_Labs\nE: ID_VENDOR_ENC=Openlight\\x20Labs\nE: ID_MODEL=CANable2_b158aa7_github.com_normaldotcom_canable2.git\nE: ID_MODEL_ENC=CANable2\\x20b158aa7\\x20github.com\\x2fnormaldotcom\\x2fcanable2.git\nE: ID_REVISION=0200\nE: ID_SERIAL=Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2068326F5052\nE: ID_SERIAL_SHORT=2068326F5052\nE: ID_TYPE=generic\nE: ID_USB_INTERFACES=:020201:0a0000:\nE: ID_USB_INTERFACE_NUM=00\nE: ID_USB_DRIVER=cdc_acm\nE: ID_USB_CLASS_FROM_DATABASE=Communications\nE: ID_USB_SUBCLASS_FROM_DATABASE=Abstract (modem)\nE: ID_PATH=pci-0000:00:14.0-usb-0:9.2:1.0\nE: ID_PATH_TAG=pci-0000_00_14_0-usb-0_9_2_1_0\nE: ID_MM_CANDIDATE=1\nE: DEVLINKS=/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2068326F5052-if00 /dev/serial/by-path/pci-0000:00:14.0-usb-0:9.2:1.0\nE: TAGS=:systemd:\n\n"
