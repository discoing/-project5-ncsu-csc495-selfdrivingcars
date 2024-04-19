#!/bin/bash

gnome-terminal --tab --title="Xbox Controller" -- bash -c 'python3 ./xbox_controller.py; exec bash'

gnome-terminal --tab --title="Driver" -- bash -c 'python3 ./driver.py; exec bash'

gnome-terminal --tab --title="Joy Node" -- bash -c 'ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0; exec bash'

gnome-terminal --tab --title="Driver" -- bash -c 'python3 ./recording_node.py; exec bash'
