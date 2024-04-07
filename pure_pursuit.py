#!/usr/bin/env python
import random
from typing import List
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


MAP_SIZE = 100

#Default: stereo image (from both L and R lenses)
VICON_TOPIC = "/vicon_pose"

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def getPoint(self):
        return self.x, self.y
    

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
    
    # Checks if a point is in the radius of the obstacle
    def checkCollision(self, point):
        radius = math.hypot(point[0] - self.x, point[1] - self.y)
        if (self.radius >= radius):
            return True
        return False
    
class PRM:

    # List of points
    map: List[Point]

    def __init__(self, start_point, end_point, obstacles):
        self.start_point = self.x_start, self.y_start
        self.end_point = self.x_end, self.y_end
        self.obstacles = obstacles



# Creates maps of random points 
    def sample_points(self, point_count):
        self.map = []
        self.map.insert(0, *self.start_point)
        for _ in range(point_count):
            while True:
                x, y = int(random.uniform(0, MAP_SIZE)), int(random.uniform(0, MAP_SIZE))
                if not self.is_obstacle((x,y)):
                    self.map.insert(len(self.map), x, y)
                    break
                
        return self.map





    # Checks if there is any obstacles at a given x,y coordinate
    def is_obstacle(self, point):

    # Checks if there is an obstacle between two x,y points
    def main(args=None):
