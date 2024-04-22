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
from a_star import Point, astar
from pure_pursuit import PurePursuit, getYawFromPose
from geometry_msgs.msg import PoseStamped
import graphics
from std_msgs.msg import String

MAP_SIZE = 4

#Default: stereo image (from both L and R lenses)
VICON_TOPIC = "/vicon_pose"
    
DEBUG_VISUAL = True

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
    
    # Checks if a point is in the radius of the obstacle
    def checkCollision(self, point : Point):
        radius = math.hypot(point.x - self.x, point.y - self.y)
        if (self.radius >= radius):
            return True
        return False
    
    def checkCollisionLine(self, start : Point, end : Point):
        # Calculate the vector representing the line segment
        dx = end.x - start.x
        dy = end.y - start.y
        
        # Parameterize the equation of the line segment
        # The equation of the line is x = x1 + t*dx, y = y1 + t*dy
        # where (x1, y1) is the start point and (x, y) is any point on the line.
        
        # Substitute the parameterized equation of the line into the equation of the circle
        # The equation of the circle is (x - cx)^2 + (y - cy)^2 = r^2
        # Substitute x and y from the line equation into the circle equation
        # (x1 + t*dx - cx)^2 + (y1 + t*dy - cy)^2 = r^2
        # Expand and simplify to get a quadratic equation in terms of t
        
        a = dx**2 + dy**2
        b = 2 * (dx * (start.x - self.x) + dy * (start.y - self.y))
        c = (start.x - self.x)**2 + (start.y - self.y)**2 - self.radius**2
        
        discriminant = b**2 - 4 * a * c
        
        if discriminant < 0:
            # No intersection
            return False
        else:
            # Check if any intersection point lies within the line segment
            t1 = (-b + math.sqrt(discriminant)) / (2 * a)
            t2 = (-b - math.sqrt(discriminant)) / (2 * a)
            if 0 <= t1 <= 1 or 0 <= t2 <= 1:
                return True
            else:
                return False

class PRM: # Probablistic Roadmap

    # List of points
    map: List[Point]

    def __init__(self, point_count, obstacles):
        self.distanceThreshold = 1
        self.obstacles : List[Obstacle] = obstacles
        self.map = self.create_map(point_count)

    # Creates maps of random (non obstacle) points 
    def create_map(self, point_count):
        # Stage 1 : Random fill
        self.map = []
        for _ in range(point_count):
            while True:
                pt = Point(random.uniform(-MAP_SIZE / 2, MAP_SIZE / 2), random.uniform(-MAP_SIZE / 2, MAP_SIZE / 2))
                if not self.collidesWithObstacle(pt):
                    self.map.append(pt)
                    break

        # Stage 2 : Make connections
        for start in self.map:
            for dest in self.map:
                if (start == dest):
                    continue
                elif start.getDistance(dest) < self.distanceThreshold  and self.canMakeConnection(start, dest):
                    start.addConnection(dest)
                    dest.addConnection(start)
                    print("Connected: " + str(start.x) + " " + str(start.y) + ", " + str(dest.x) + " " + str(dest.y))

        return self.map

    def findNearestPoint(self, target):
        nearest : Point = self.map[0]
        last_distance = 10000
        for x in self.map:
            dist = target.getDistance(x)
            if dist < last_distance and len(x.destination) > 1:
                last_distance = dist
                nearest = x
        return nearest

    def canMakeConnection(self, start, dest):
        for obstacle in self.obstacles:
            if obstacle.checkCollisionLine(start, dest) == True:
                return False
        return True

    # Checks if there is any obstacles at a given x,y coordinate
    def collidesWithObstacle(self, point : Point):
        for obstacle in self.obstacles:
            if obstacle.checkCollision(point):
                return True
        return False
    
    # ONLINE

    def makePathToPoint(self, curr_point, destination : Point):
        # Do A*
        nearest = self.findNearestPoint(curr_point)
        result = astar(nearest, destination, self.map)
        return result

last_pose : PoseStamped = None
obstacles = [(Obstacle(1.530, 0.610, .4)), (Obstacle(0.4556, 1.2915, 0.4)), (Obstacle(0.05, -1.1906, 0.4)),
             Obstacle(-1.479, 0.669, 0.4), Obstacle(-.883, -.995, 0.4)]
def updateVicon(data : PoseStamped):
    global last_pose
    last_pose = data
    #print(last_pose)

def convertToDebugWindowPoint(p):
    return graphics.Point((0.5 + p.x / MAP_SIZE) * 500, (0.5 + p.y / MAP_SIZE) * 500)

OFFSET_SCALAR = 40
def convertAngleToOffset(angle, base : Point=None):
    pt = graphics.Point(math.cos(angle) * OFFSET_SCALAR, math.sin(angle) * OFFSET_SCALAR)
    if base is not None:
        pt.x += base.x
        pt.y += base.y
    return pt

destinations = {
    "eb1" : obstacles[0],
    "eb2" : obstacles[1],
    "eb3" : obstacles[2],
    "hunt" : obstacles[3],
    "textiles" : obstacles[4]
}
OBSTACLE_ACQUIRE_RAD = 0.01
def incomingUserCommand(data : String):
    data = data.data
    # EB1, EB2, EB3, Hunt, Textiles
    global destination, finishedPath, road, path_refresh_needed
    command : Obstacle = destinations.get(data.lower())
    if command is None:
        print("Unsure what " + data + " is")
        return 
    if last_pose is None or last_pose.pose is None:
        return

    car_point = Point(last_pose.pose.position.x, last_pose.pose.position.y)

    # Project nearest point around the obstacle
    projected = Point(car_point.x - command.x, car_point.y - command.y)
    length = math.sqrt(projected.x**2 + projected.y**2)
    scalar = (command.radius + OBSTACLE_ACQUIRE_RAD) / length
    projected.x *= scalar
    projected.y *= scalar
    projected.x += command.x
    projected.y += command.y
    
    destination = road.findNearestPoint(projected)

    finishedPath = road.makePathToPoint(car_point, destination)

    if finishedPath is None:
        destination = None

    print ("made path with waypoints " + str(len(finishedPath)))
    path_refresh_needed = True

def updatePathDebug(win, currentPathsLines : List[Point]):
    if currentPathsLines is not None:
        for line in currentPathsLines:
            line.undraw()
    currentPathsLines.clear()

    # The finished path
    if finishedPath is not None:
        for i in range(0, len(finishedPath) - 1):
            l = graphics.Line(convertToDebugWindowPoint(finishedPath[i]), convertToDebugWindowPoint(finishedPath[i + 1]))
            l.setFill("blue")
            l.setArrow("last")
            l.draw(win)
            currentPathsLines.append(l)
    win.update()

road = None
finishedPath = None
destination = None
path_refresh_needed = False
def main(args=None):
    global road, finishedPath, destination, path_refresh_needed
    currentPathsLines = []
    
    rclpy.init(args=args)

    node = Node("path_follower")

    node.create_subscription(PoseStamped, VICON_TOPIC, updateVicon, 5)
    node.create_subscription(String, "/location_topic", incomingUserCommand, 2)
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())

    rate.sleep() # Wait for topic data to stabalize
    print("Waiting for VICON update")
    while last_pose == None:
        rate.sleep() # Wait for first pose
    print("VICON update received")

    # Build map and add in current vicon point as initial
    start_point = Point(last_pose.pose.position.x, last_pose.pose.position.y)
    road = PRM(120, obstacles)
    print("Finish building map")

    # Draw the graph
    if DEBUG_VISUAL:
        win = graphics.GraphWin(width = 500, height = 500)

        # Obstacles
        for o in obstacles:
            c = graphics.Circle(convertToDebugWindowPoint(o), o.radius / MAP_SIZE * 500)
            c.setFill("yellow")
            c.draw(win)

        # Waypoints
        color = graphics.color_rgb(200, 200, 200)
        for p in road.map:
            pDebug = convertToDebugWindowPoint(p)
            c = graphics.Circle(pDebug, 4)
            c.setFill("black")
            c.draw(win)

            for dest in p.destination:
                l = graphics.Line(pDebug, convertToDebugWindowPoint(dest))
                l.setFill(color)
                l.draw(win)
        # The car
        last_debug_updated = convertToDebugWindowPoint(start_point)
        car = graphics.Circle(last_debug_updated, 8)
        car.setFill("red")
        car.draw(win)
        # The car's heading
        yaw = getYawFromPose(last_pose)
        heading = graphics.Line(last_debug_updated, convertAngleToOffset(yaw, last_debug_updated))
        heading.setFill("orange")
        heading.setArrow("last")
        heading.draw(win)

        # Lookahead target
        lookahead_point = graphics.Circle(graphics.Point(0, 0), 12)
        lookahead_point.setFill("yellow")
        lookahead_point.draw(win)
        lookahead_point.center = graphics.Point(0, 0)

        updatePathDebug(win, currentPathsLines)

    # Build path follower
    follower = PurePursuit()
    steer = node.create_publisher(Float32, "/cv_steer", 10)
    manual_pub = node.create_publisher(Float32, "/cv_throttle", 10)
    steeringCommand : Float32 = Float32()
    throttleCommand : Float32 = Float32()

    while True:
        
        throttle = 0.0
        lookahead = None
        if destination is not None:
            # Pure pursuit
            command, lookahead_res = follower.followPath(last_pose, finishedPath)
            lookahead = lookahead_res
            #print(command)
            steeringCommand.data = command
            steer.publish(steeringCommand)
            throttle = 0.88

            # Stopping
            point = last_pose.pose.position
            if (point.x - destination.x)**2 + (point.y - destination.y)**2 < 0.08: # stop threshold
                destination = None

        # Publish throttle
        throttleCommand.data = throttle
        manual_pub.publish(throttleCommand)

        if DEBUG_VISUAL:
            # Update car point and heading in window
            curr = convertToDebugWindowPoint(Point(last_pose.pose.position.x, last_pose.pose.position.y))
            dx = curr.x - last_debug_updated.x
            dy = curr.y - last_debug_updated.y
            car.move(dx, dy)
            heading.move(dx, dy)
            # convertAngleToOffset(getYawFromPose(last_pose), curr)

            if path_refresh_needed:
                updatePathDebug(win, currentPathsLines)
                path_refresh_needed = False

            if lookahead is not None:
                curr_look = convertToDebugWindowPoint(Point(lookahead.x, lookahead.y))
                dx = curr_look.x - lookahead_point.center.x
                dy = curr_look.y - lookahead_point.center.y
                lookahead_point.move(dx, dy)
                lookahead_point.center = curr_look
                
            win.update()
            last_debug_updated = curr
    win.close()
    
if __name__ == '__main__':
    main()