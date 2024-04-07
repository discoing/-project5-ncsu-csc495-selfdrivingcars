#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import struct
import can
import threading
import time

throttle = 0
steer = 0

def throttle_callback(data : Int64):
    global throttle
    throttle = data.data

def steering_callback(data : Int64):
    global steer
    steer = data.data

def main(args=None):
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate= 250000)
    
    rclpy.init(args=args)
    node = Node("driver")

    manual_throttle = node.create_subscription(Int64, "/manual_throttle", throttle_callback, 10)
    manual_steering = node.create_subscription(Int64, "/manual_steering", steering_callback, 10)
    
    thread = threading.Thread(target=rclpy. spin, args=(node, ), daemon=True)
    thread.start()
 
    rate = node.create_rate(20, node.get_clock())
    while rclpy.ok():

        print('throttle: %d, steering: %d' % (throttle, steer))
        try:
            # DO NOT COMPUTE THE PWM VALUES IN ORIN. Just send the raw command values. 
            
            can_data = struct.pack('>hhI', throttle, steer, 0)
            msg = can.Message(arbitration_id=0x1, data = can_data, is_extended_id=False)
            bus.send(msg)
        except Exception as error:
            print("An exception occurred:", error)
        finally:
            rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

	
if __name__ == '__main__':
	main()