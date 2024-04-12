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
from geometry_msgs.msg import PoseStamped


MAP_SIZE = 100

#Default: stereo image (from both L and R lenses)
VICON_TOPIC = "/vicon_pose"

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.destination = []
    
    def getPoint(self):
        return self.x, self.y
    
    def addConnection(self, other_point):
        self.destination.append(other_point)

    # Euclidean distance of given point
    def getDistance(self, other):
        return math.hypot((self.x - other.x), (self.y - other.y))
    
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
    
    def checkCollisionLine(self, start : Point, end : Point):
        line_vector = (end.x - start.x, end.y - start.y)

        circle_to_start = (start.x - self.x, start.y - self.y)

        dot_product = line_vector[0] * circle_to_start[0] + line_vector[1] * circle_to_start[1]

        line_length_squared = line_vector[0]**2 + line_vector[1]**2

        closest_distance_squared = (dot_product**2) / line_length_squared

        if closest_distance_squared <= self.radius**2:
            return True 
        else:
            return False
    
class PRM: # Probablistic Roadmap

    # List of points
    map: List[Point]

    def __init__(self, point_count, obstacles, start_point):
        self.distanceThreshold = 4
        self.obstacles = obstacles
        self.start_point = start_point
        self.map = self.create_map(point_count)
        self.temp_point = Point()

    # Creates maps of random (non obstacle) points 
    def create_map(self, point_count):
        # Stage 1 : Random fill
        self.map = []
        self.map.insert(0, self.start_point)
        for _ in range(point_count):
            while True:
                x, y = int(random.uniform(0, MAP_SIZE)), int(random.uniform(0, MAP_SIZE))
                if not self.collidesWithObstacle((x,y)):
                    self.map.insert(len(self.map), x, y)
                    break

        # Stage 2 : Make connections
        for start in self.map:
            for dest in self.map:
                if self.getDistance(start, dest) < self.distanceThreshold and self.canMakeConnection(start, dest):
                    start.addConnection(dest)
                    dest.addConnection(start)

        return self.map

    def canMakeConnection(self, start, dest):
        for obstacle in self.obstacles:
            if obstacle.checkCollisionLine(start, dest):
                return False
        return True

    # Checks if there is any obstacles at a given x,y coordinate
    def collidesWithObstacle(self, point):
        for obstacle in self.obstacles:
            if obstacle.checkCollision(point):
                return True
        return False
    
    # ONLINE

    def makePathToPoint(point : Point):
        # Do A*
        
        return # TODO
    
last_pose : PoseStamped = None
obstacles = [Obstacle(0, 0, 0.2), Obstacle(1, 2, 0.2), Obstacle(-1, -0.1, 0.2)]
def updateVicon(data : PoseStamped):
    global last_pose
    last_pose = data
    print(last_pose)

def main(args=None):
    rclpy.init(args=args)

    node = Node("path_follower")

    node.create_subscription(PoseStamped, VICON_TOPIC, updateVicon, 5)
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())

    rate.sleep() # Wait for topic data to stabalize
    print("Waiting for VICON update")
    while last_pose == None:
        rate.sleep() # Wait for first pose
    print("VICON update received")

    road = PRM(100, obstacles, Point(last_pose.pose.position.x, last_pose.pose.position.y))
    print("Finish building map")

if __name__ == '__main__':
    main()