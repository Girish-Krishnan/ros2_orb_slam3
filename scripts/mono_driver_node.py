#!/usr/bin/env python3

"""
Python node for the MonocularMode cpp node.

Author: Azmyin Md. Kamal
Date: 01/01/2024

Requirements
* Dataset must be configured in EuRoC MAV format
* Paths to dataset must be set before bulding (or running) this node
* Make sure to set path to your workspace in common.hpp

Command line arguments
-- settings_name: EuRoC, TUM2, KITTI etc; the name of the .yaml file containing camera intrinsics and other configurations
-- image_seq: MH01, V102, etc; the name of the image sequence you want to run

MODIFICATIONS by Girish Krishnan:
Modified ROS node to accept image messages from RealSense camera, instead of reading images from disk
"""

# Imports
#* Import Python modules
import sys # System specific modules
import os # Operating specific functions
import glob
import time # Python timing module
import copy # For deepcopying arrays
import shutil # High level folder operation tool
from pathlib import Path # To find the "home" directory location
import argparse # To accept user arguments from commandline
import natsort # To ensure all images are chosen loaded in the correct order
import yaml # To manipulate YAML files for reading configuration files
import copy # For making deepcopies of openCV matrices, python lists, numpy arrays etc.
import numpy as np # Python Linear Algebra module
import cv2 # OpenCV

#* ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# If you have more files in the submodules folder
# from .submodules.py_utils import fn1 # Import helper functions from files in your submodules folder

# Import a custom message interface
# from your_custom_msg_interface.msg import CustomMsg #* Note the camel caps convention

# Import ROS2 message templates
from sensor_msgs.msg import Image # http://wiki.ros.org/sensor_msgs
from std_msgs.msg import String, Float64 # ROS2 string message template
from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array

#* Class Definition
class MonoDriver(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)

        # Initialize parameters to be passed from the command line (or launch file)
        self.declare_parameter("settings_name", "EuRoC")
        self.declare_parameter("image_seq", "NULL")

        # Parse values sent by command line
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.image_seq = str(self.get_parameter('image_seq').value)

        # Setup ROS2 publishers and subscribers
        self.publish_exp_config_ = self.create_publisher(String, "/mono_py_driver/experiment_settings", 1)
        self.subscribe_exp_ack_ = self.create_subscription(String, "/mono_py_driver/exp_settings_ack", self.ack_callback, 10)
        self.publish_img_msg_ = self.create_publisher(Image, "/mono_py_driver/img_msg", 1)
        self.publish_timestep_msg_ = self.create_publisher(Float64, "/mono_py_driver/timestep_msg", 1)

        # Subscribe to the RealSense camera feed
        self.image_sub_ = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)

        # Define a CvBridge object
        self.br = CvBridge()

        self.send_config = True
        self.frame_id = 0

        print(f"MonoDriver initialized, attempting handshake with CPP node")

    def ack_callback(self, msg):
        print(f"Got ack: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False

    def handshake_with_cpp_node(self):
        if self.send_config:
            msg = String()
            msg.data = self.settings_name
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    def image_callback(self, img_msg):
        try:
            cv_image = self.br.imgmsg_to_cv2(img_msg, "bgr8")
            self.frame_id += 1
            self.run_py_node(cv_image, img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9)
        except CvBridgeError as e:
            print(e)

    def run_py_node(self, img, timestamp):
        img_msg = self.br.cv2_to_imgmsg(img, encoding="bgr8")
        timestep_msg = Float64()
        timestep_msg.data = timestamp

        try:
            self.publish_timestep_msg_.publish(timestep_msg)
            self.publish_img_msg_.publish(img_msg)
        except CvBridgeError as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    n = MonoDriver("mono_py_node")
    rate = n.create_rate(20)

    while n.send_config:
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)

    print(f"Handshake complete")

    rclpy.spin(n)

    cv2.destroyAllWindows()
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
