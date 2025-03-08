#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import geometry_msgs.msg
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler
import tf2_ros
from pynput import keyboard
from ai2thor.controller import Controller
import numpy as np
import threading

# Initialize AI2-THOR Controller
controller = Controller(
    agentMode="locobot",
    visibilityDistance=3.0,
    scene="FloorPlan_Train10_2",
    gridSize=0.1,
    movementGaussianSigma=0.005,
    rotateStepDegrees=90,
    rotateGaussianSigma=0.5,
    renderDepthImage=True,
    renderInstanceSegmentation=False,
    width=640,
    height=480,
    fieldOfView=60,  # Vertical field of view for the depth camera
)

# ROS Publishers
rgb_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)
depth_pub = rospy.Publisher(
    "/camera/aligned_depth_to_color/image_raw", Image, queue_size=10
)
camera_info_pub = rospy.Publisher(
    "/camera/color/camera_info", CameraInfo, queue_size=10
)
bridge = CvBridge()  # Used for converting between OpenCV images and ROS messages
static_broadcaster = tf2_ros.StaticTransformBroadcaster()

# Create a lock to ensure thread-safe access to controller.step()
controller_lock = threading.Lock()

# Variable to track whether RoboTHOR control is enabled
control_enabled = False  # Start with control disabled

ACTION_DICT = {
    "move": {
        "move_magnitude": 0.1,
        "w": "MoveAhead",
        "s": "MoveBack",
        "a": "MoveLeft",
        "d": "MoveRight",
    },
    "look": {
        "look_degrees": 30,
        keyboard.Key.up: "LookUp",
        keyboard.Key.down: "LookDown",
    },
    "rotate": {
        "rotate_degrees": 10,
        keyboard.Key.left: "RotateLeft",
        keyboard.Key.right: "RotateRight",
    },
}


# Function to publish RGB and depth images, CameraInfo, and TF with the same timestamp
def publish_all():
    with controller_lock:  # Ensure thread-safe access to controller
        try:
            # Get the current ROS time
            current_time = rospy.Time.now()

            # Get the current frame's RGB and depth images
            event = controller.step(action="Pass")
            rgb_image = event.frame  # RGB image (numpy array)
            depth_image = event.depth_frame  # Depth image (numpy array, float32)

            # Convert depth image to 32FC1 format
            depth_image_32fc1 = depth_image.astype(
                np.float32
            )  # AI2-THOR depth is already in meters

            # Convert to ROS Image messages
            rgb_msg = bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
            depth_msg = bridge.cv2_to_imgmsg(depth_image_32fc1, encoding="32FC1")

            # Add the same timestamp to all messages
            rgb_msg.header.stamp = current_time
            rgb_msg.header.frame_id = "camera_color_frame"
            depth_msg.header.stamp = current_time
            depth_msg.header.frame_id = "camera_aligned_depth_to_color_frame"

            # Publish RGB and depth images
            rgb_pub.publish(rgb_msg)
            depth_pub.publish(depth_msg)

            # Create and publish CameraInfo
            camera_info_msg = get_camera_info()
            camera_info_msg.header.stamp = current_time
            camera_info_pub.publish(camera_info_msg)

            # Publish static transforms
            publish_static_tf(current_time)

        except Exception as e:
            rospy.logerr(f"Error during publish_all: {e}")


def get_camera_info():
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "camera_color_optical_frame"  # Match frame_id
    camera_info_msg.height = 480  # Match height
    camera_info_msg.width = 640  # Match width
    camera_info_msg.distortion_model = "plumb_bob"  # Match distortion model

    # Match distortion coefficients
    camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]

    # Match intrinsic camera matrix (K)
    camera_info_msg.K = [
        615.973876953125,
        0.0,
        321.73553466796875,
        0.0,
        614.9360961914062,
        238.9122314453125,
        0.0,
        0.0,
        1.0,
    ]

    # Match rectification matrix (R)
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    # Match projection matrix (P)
    camera_info_msg.P = [
        615.973876953125,
        0.0,
        321.73553466796875,
        0.0,
        0.0,
        614.9360961914062,
        238.9122314453125,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]

    # Match binning and region of interest (ROI)
    camera_info_msg.binning_x = 0
    camera_info_msg.binning_y = 0
    camera_info_msg.roi.x_offset = 0
    camera_info_msg.roi.y_offset = 0
    camera_info_msg.roi.height = 0
    camera_info_msg.roi.width = 0
    camera_info_msg.roi.do_rectify = False

    return camera_info_msg


def publish_static_tf(current_time):
    transforms = []

    # Transform 1: camera_link -> camera_depth_frame
    tf1 = geometry_msgs.msg.TransformStamped()
    tf1.header.stamp = current_time
    tf1.header.frame_id = "camera_link"
    tf1.child_frame_id = "camera_depth_frame"
    tf1.transform.translation.x = 0.0
    tf1.transform.translation.y = 0.0
    tf1.transform.translation.z = 0.0
    tf1.transform.rotation.x = 0.0
    tf1.transform.rotation.y = 0.0
    tf1.transform.rotation.z = 0.0
    tf1.transform.rotation.w = 1.0
    transforms.append(tf1)

    # Transform 2: camera_depth_frame -> camera_depth_optical_frame
    tf2 = geometry_msgs.msg.TransformStamped()
    tf2.header.stamp = current_time
    tf2.header.frame_id = "camera_depth_frame"
    tf2.child_frame_id = "camera_depth_optical_frame"
    tf2.transform.translation.x = 0.0
    tf2.transform.translation.y = 0.0
    tf2.transform.translation.z = 0.0
    tf2.transform.rotation.x = -0.5
    tf2.transform.rotation.y = 0.5
    tf2.transform.rotation.z = -0.5
    tf2.transform.rotation.w = 0.5
    transforms.append(tf2)

    # Transform 3: camera_link -> camera_color_frame
    tf3 = geometry_msgs.msg.TransformStamped()
    tf3.header.stamp = current_time
    tf3.header.frame_id = "camera_link"
    tf3.child_frame_id = "camera_color_frame"
    tf3.transform.translation.x = -0.00017157249385491014
    tf3.transform.translation.y = 0.014982420019805431
    tf3.transform.translation.z = -5.048635648563504e-05
    tf3.transform.rotation.x = 0.0029460720252245665
    tf3.transform.rotation.y = 0.0011356002651154995
    tf3.transform.rotation.z = 0.0031310124322772026
    tf3.transform.rotation.w = 0.9999901056289673
    transforms.append(tf3)

    # Transform 4: camera_aligned_depth_to_color_frame -> camera_color_optical_frame
    tf4 = geometry_msgs.msg.TransformStamped()
    tf4.header.stamp = current_time
    tf4.header.frame_id = "camera_aligned_depth_to_color_frame"
    tf4.child_frame_id = "camera_color_optical_frame"
    tf4.transform.translation.x = 0.0
    tf4.transform.translation.y = 0.0
    tf4.transform.translation.z = 0.0
    tf4.transform.rotation.x = -0.5
    tf4.transform.rotation.y = 0.5
    tf4.transform.rotation.z = -0.5
    tf4.transform.rotation.w = 0.5
    transforms.append(tf4)

    # Transform 5: camera_link -> camera_aligned_depth_to_color_frame
    tf5 = geometry_msgs.msg.TransformStamped()
    tf5.header.stamp = current_time
    tf5.header.frame_id = "camera_link"
    tf5.child_frame_id = "camera_aligned_depth_to_color_frame"
    tf5.transform.translation.x = -0.00017157249385491014
    tf5.transform.translation.y = 0.014982420019805431
    tf5.transform.translation.z = -5.048635648563504e-05
    tf5.transform.rotation.x = 0.0029460720252245665
    tf5.transform.rotation.y = 0.0011356002651154995
    tf5.transform.rotation.z = 0.0031310124322772026
    tf5.transform.rotation.w = 0.9999901056289673
    transforms.append(tf5)

    # Publish all transforms
    static_broadcaster.sendTransform(transforms)


# Robot control function
def control_robot(key):
    global control_enabled
    if not control_enabled:  # Check if control is enabled
        rospy.loginfo("Control is disabled. Ignoring key press.")
        return

    with controller_lock:  # Ensure thread-safe access to the controller
        try:
            # Check if a key for movement was pressed
            if key.char.lower() in ACTION_DICT["move"].keys():
                action = ACTION_DICT["move"][key.char.lower()]
                move_magnitude = ACTION_DICT["move"]["move_magnitude"]
                controller.step(action=action, moveMagnitude=move_magnitude)
                rospy.loginfo(f"{action}, moveMagnitude: {move_magnitude}")
        except AttributeError:
            # Check for non-character keys
            if key in ACTION_DICT["look"].keys():
                action = ACTION_DICT["look"][key]
                look_degrees = ACTION_DICT["look"]["look_degrees"]
                controller.step(action=action)
                rospy.loginfo(f"{action}, lookDegrees: {look_degrees}")
            elif key in ACTION_DICT["rotate"].keys():
                action = ACTION_DICT["rotate"][key]
                rotate_degrees = ACTION_DICT["rotate"]["rotate_degrees"]
                controller.step(action=action, degrees=rotate_degrees)
                rospy.loginfo(f"{action}, rotateDegrees: {rotate_degrees}")


# Keyboard event handlers
def on_press(key):
    global control_enabled

    # Toggle control on/off with CapsLock
    if key == keyboard.Key.caps_lock:
        control_enabled = not control_enabled
        rospy.loginfo(f"Control {'enabled' if control_enabled else 'disabled'}")
        return

    # Handle robot control
    control_robot(key)


def on_release(key):
    if key == keyboard.Key.esc:  # Exit the program when ESC is pressed
        rospy.signal_shutdown("User requested shutdown.")
        return False


# ROS node main function
def main():
    rospy.init_node("robothor", anonymous=True)
    rospy.loginfo(
        "RoboTHOR ROS node started. Use 'WASD' to move, arrow keys to look/rotate, ESC to exit. CapsLock to enable/disable control."
    )

    # Run the keyboard listener in a separate thread
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # Continuous publishing loop
    rate = rospy.Rate(100)  # Publish data at 100Hz
    while not rospy.is_shutdown():
        publish_all()
        rate.sleep()

    listener.join()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
