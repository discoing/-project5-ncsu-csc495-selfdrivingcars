#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import threading
import time
import math
import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import Int64
from std_msgs.msg import Float32
import time

#Default: stereo image (from both L and R lenses)
VICON_TOPIC = "/vicon_pose"


def main(args=None):
