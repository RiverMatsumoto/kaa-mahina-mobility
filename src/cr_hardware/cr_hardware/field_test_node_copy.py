#!/usr/bin/env python3

#/home/roselab/miniconda3/envs/ml_rover/bin/python3
# -*- coding: utf-8 -*- #
"""
ʕっ•ᴥ•ʔっ
@authors: jen & sapph

last updated: aug 19 2024 09:22
"""

#ros imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# basics
import os
#os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.stats as ss

# specialized
from numpy.linalg import inv
from osgeo import gdal
from PIL import Image
import threading
import torch
import gpytorch
import sklearn.metrics
import cv2

# other
from serial import Serial, SerialException
from scipy.stats import ks_2samp
from sklearn.metrics import mean_squared_error
import serial
#import pyautogui
import shutil
import pathlib
import glob
#import keyboard
import datetime
import random
import csv
import pickle
import re
import math
import time
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StringProcessorNode(Node):

    def __init__(self):
        super().__init__('string_processor_node')

        # Create a subscriber
        self.subscription = self.create_subscription(
            String,
            '/input_string',  # Topic name to subscribe to
            self.listener_callback,
            10)  # QoS profile depth

        # Create a publisher
        self.publisher = self.create_publisher(
            String,
            '/output_string',  # Topic name to publish to
            10)  # QoS profile depth
        self.create_timer(1, self.publish)
    
    def publish(self):
        msg = String()
        msg.data = "hello"
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        # Process the incoming message (convert it to uppercase)
        processed_string = msg.data.upper()

        # Create a new message and publish it
        new_msg = String()
        new_msg.data = processed_string
        self.publisher.publish(new_msg)

        # Log the received and sent messages
        self.get_logger().info(f'Received: "{msg.data}", Published: "{new_msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = StringProcessorNode()

    # Keep the node alive
    rclpy.spin(node)

    # Shutdown the ROS client library
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()