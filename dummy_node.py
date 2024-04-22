import pyaudio
from faster_whisper import WhisperModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tempfile
import wave
import threading
import os
import re

rclpy.init()

node = Node("dummy")
location_publisher = node.create_publisher(String, '/location_topic', 10)