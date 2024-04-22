from typing import List
from a_star import Point
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from tf_transformations import euler_from_quaternion

def getYawFromPose(pose : PoseStamped):
    orientation_quat = pose.pose.orientation
    quat = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(quat)    # the Euler angles are in radians
    return yaw

class PurePursuit:
    def __init__(self):
        self.lookAheadAmount = 1
        self.P = 0.65

    def angleDifference(self, one, two):
        diff = two - one
        if diff > math.pi:
            diff -= math.tau
        elif diff < -math.pi:
            diff += math.tau
        return diff
    
    def PID(self, error):
        return error * self.P

    def followPath(self, pose : PoseStamped, path : List[Point]):
        # Calculate lookahead
        pos = pose.pose.position
        yaw = getYawFromPose(pose)

        # A) create a point at a right angle and rotate directly by the quaternion
        # B) Use sin and cos to calculate point
        # C) rip the next point from the list and rotate it into the car fram
        
        # serches for nearest point to car assume their are multiple points within one lookahead distance
        # assuming dense waypoints
        closestPointIndex = 0
        closestPointDist = math.inf
        for i in range(len(path)):
            pt = path[i] 
            distanceToPointSqrd = (pt.x - pos.x)**2 + (pt.y - pos.y)**2
            if(distanceToPointSqrd < closestPointDist):
                closestPointIndex = i
                closestPointDist = distanceToPointSqrd
        
        lookAheadIndex = closestPointIndex + self.lookAheadAmount
        lookAheadIndex = min(lookAheadIndex, len(path) - 1)

        # calcualte angle theta between current car positon and lookahead point
        # Delta x between car and lookahead
        x = (path[lookAheadIndex].x - pos.x)
        # Delta y between car and lookahead
        y = (path[lookAheadIndex].y - pos.y)

        theta = math.atan2(y, x)

        # Get the differences between the desired and actual angle
        error = self.angleDifference(yaw, theta)
        print(str(error * 180 / math.pi) + " " + str(theta * 180 / math.pi))
        
        # Map to PID
        return (self.PID(error), path[lookAheadIndex])