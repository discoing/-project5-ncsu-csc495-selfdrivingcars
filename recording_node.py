import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave
import threading
import subprocess
import os

class AudioRecordingNode(Node):
    def __init__(self):
        super().__init__('AudioRecordingNode')
        self.subscription = self.create_subscription(String, "/recording_control", self.recording_control_callback, 10)
        self.is_recording = False
        self.frames = []
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=pyaudio.paInt16, channels=1, rate=44100, input=True, frames_per_buffer=1024)

    def recording_control_callback(self, msg):
        if msg.data == "toggle":
            if not self.is_recording:
                self.start_recording()
            else:
                self.stop_recording()

    def start_recording(self):
        self.is_recording = True
        self.frames = []
        self.get_logger().info('Recording started...')
        threading.Thread(target=self.record_audio).start()

    def record_audio(self):
        while self.is_recording:
            data = self.stream.read(1024)
            self.frames.append(data)

    def stop_recording(self):
        self.is_recording = False
        self.get_logger().info('Recording stopped.')
        # Save the recorded data as a WAV file
        wave_file = wave.open('file.wav', 'wb')
        wave_file.setnchannels(1)
        wave_file.setsampwidth(self.audio.get_sample_size(pyaudio.paInt16))
        wave_file.setframerate(44100)
        wave_file.writeframes(b''.join(self.frames))
        wave_file.close()
        subprocess.call(['ffmpeg', '-i', 'file.wav', 'file.mp3'])
        os.remove('file.wav')

    # Makes sure that the mic index referenced is the correct device
    def find_mic_index(self, mic):
        num_devices = self.audio.get_device_count
        for i in range(num_devices):
            mic_info = self.audio.get_device_info_by_host_api_device_index(i)
            if mic in mic_info.get('name'):
                return i
            raise Exception("Mic not found")

def main(args=None):
    rclpy.init(args=args)
    node = AudioRecordingNode()
    rclpy.spin(node)
    node.stream.stop_stream()
    node.stream.close()
    node.audio.terminate()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
