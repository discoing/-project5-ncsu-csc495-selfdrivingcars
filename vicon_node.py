#!/usr/bin/env python
import sys
import math
import rclpy
from rclpy.node import Node
from pyvicon_datastream import tools
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
import math
import threading
import tf_transformations
import tf2_ros

VICON_TRACKER_IP = "wolfwagen-win01.csc.ncsu.edu"

#####################
OBJECT_NAME = "wolfwagen07"    # XY: your vehicle number
#####################


#This will try to connect to the VICON TRACKER
vicontracker = tools.ObjectTracker(VICON_TRACKER_IP)
pose = None

def quat_from_euler(roll, pitch, yaw):
    q_orig = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    return q_orig

def periodic_loop():
    global pose

    raw_data = vicontracker.get_position(OBJECT_NAME)
    if raw_data is False:
        print(f"Cannot find object {OBJECT_NAME}")
        return

    _, _, pos_data = raw_data
    
    # print(f"Raw position data: {pos_data}")
    print("")
    if pos_data != []:
        xyz = pos_data[0][2:5]
        pos_data[0][7] += math.pi/2.0   #currently, negative x direction = 0 degree. So, rotate
        orientation = pos_data[0][7]
        orientation_angles = pos_data[0][5:]
        pose = quat_from_euler(orientation_angles[0], orientation_angles[1], orientation_angles[2])
 
        print(f"Position: {xyz}")
        print(f"Orientation (RPY): {orientation_angles[0]}, {orientation_angles[1]}, {orientation_angles[2]}")
        print(f"Orientation Rad/Deg.: {orientation:.4f}/{math.degrees(orientation):.4f}")

        return xyz, pose
    else:
        print("no vicon data!")

    print("----")

def main(args=None):
    global pose

    if not vicontracker.is_connected:
        sys.exit(0)

    rclpy.init(args=args)
    node = Node("vicon_node")
    vicon_publisher = node.create_publisher(PoseStamped , 'vicon_pose' , 10)
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())

    while rclpy.ok():
        try:
            xyz, pose = periodic_loop()
            data = Point()
            quat = Quaternion()

            retVal = PoseStamped()

            br = tf2_ros.TransformBroadcaster(node)
            t = TransformStamped()

            data.x = xyz[0] / 1000
            data.y = xyz[1] / 1000
            data.z = xyz[2] / 1000

            quat.w = pose[3]
            quat.x = pose[0]
            quat.y = pose[1]
            quat.z = pose[2]
            
            retVal.pose.position = data
            retVal.pose.orientation = quat

            # print(pose)

            t.header.stamp = node.get_clock().now().to_msg() 
            t.header.frame_id = "map"
            t.child_frame_id = 'laser'
            

            t.transform.translation.x = data.x
            t.transform.translation.y = data.y
            t.transform.translation.z = data.z

            t.transform.rotation.x = quat.x 
            t.transform.rotation.y = quat.y
            t.transform.rotation.z = quat.z
            t.transform.rotation.w = quat.w    

            br.sendTransform(t)

            retVal.header.stamp = node.get_clock().now().to_msg() 
            retVal.header.frame_id = "map"
            try:

                vicon_publisher.publish(retVal)

            except Exception as e:
                print("Failed to publish: ")
                print(e)
        except TypeError as e:
            print("Car not being picked up by camera, re-run the program, or try to make sure the car is in view of the cameras.")

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
