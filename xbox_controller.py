#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
from std_msgs.msg import Float32
from std_msgs.msg import String
import threading
import curses


stdscr = curses.initscr()

# Throttle should be bounded between [-10, +4]
MAX_MANUAL_THROTTLE_FORWARD = 20
MAX_MANUAL_THROTTLE_REVERSE = 20

# Steering should be bounded between [-100, +100]

is_manual = True

steering = 0
throttle = 0

def joy_callback(data : Joy, recording_pub):
	global steering, throttle, is_manual

	if data.buttons[0]:
		is_manual = not is_manual

	if is_manual:
		throttle = data.axes[1]
		steering = data.axes[2]
	else:
		if data.axes[6] and throttle > 0:
			if throttle == 0.80:
				throttle = 0
			else:
				throttle -= 0.01
		elif data.axes[7] and throttle < 1.0:
			if throttle == 0:
				throttle = 0.85
			else:
				throttle += 0.01

	if data.buttons[1]:
		recording_pub.publish(String(data="toggle"))
		print


def lane_following_callback(data : Float32):
	global is_manual, steering, throttle
	if is_manual == False:
		steering = data.data
		steering = max(min(steering, 1.0), -1.0)


#??
def remap( val : float, lowerStart : float, upperStart : float, lowerEnd : float, upperEnd : float):
	return int(((val - lowerStart) / (upperStart - lowerStart)) * (upperEnd - lowerEnd) + lowerEnd)

def main(args=None):

	rclpy.init(args=args)
	node = Node("xbox_controller_node")


	## ------------
	## YOUR CODE
	# Subscribe to the 'joy' topic
	# Create publishers for the manual_throttle and manual_steering commands
	## ------------

	node.create_subscription(Joy, "/joy", lambda data: joy_callback(data, recording_pub), 10)
	node.create_subscription(Float32, "/cv_steer", lane_following_callback, 10)
	manual_pub = node.create_publisher(Int64, "/manual_throttle", 10)
	manual_steering = node.create_publisher(Int64, "/manual_steering", 10)
	recording_pub = node.create_publisher(String, "/recording_control", 10)


	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(20, node.get_clock())
        
	while rclpy.ok():

		try:
			# Publish throttle
			msg : Int64 = Int64()
			if (throttle < 0):
				msg.data = int(throttle * MAX_MANUAL_THROTTLE_REVERSE)
			else:
				msg.data = int(throttle * MAX_MANUAL_THROTTLE_FORWARD)

			manual_pub.publish(msg)

			# Publish steering
			msg = Int64()
			msg.data = remap(steering, 1.0, -1.0, -100, 100)
			manual_steering.publish(msg)


			stdscr.refresh()
			stdscr.addstr(1, 25, 'Xbox Controller       ')
			stdscr.addstr(2, 25, 'Throttle: %.2f  ' % throttle)
			stdscr.addstr(3, 25, 'Steering: %.2f  ' % steering)

			rate.sleep()
		except KeyboardInterrupt:
			curses.endwin()
			print("Ctrl+C captured, ending...")
			break
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()
