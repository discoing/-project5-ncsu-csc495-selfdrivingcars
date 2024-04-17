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
        super().__init__('audio_recording_node')
        self.subscription = self.create_subscription(String, "/recording_control", self.recording_control_callback, 10)
        self.is_recording = False
        self.frames = []
        self.pyaudio_instance = pyaudio.PyAudio()
        self.stream = self.pyaudio_instance.open(format=pyaudio.paInt16, channels=1, rate=44100, input=True, frames_per_buffer=1024)

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
        wave_file.setsampwidth(self.pyaudio_instance.get_sample_size(pyaudio.paInt16))
        wave_file.setframerate(44100)
        wave_file.writeframes(b''.join(self.frames))
        wave_file.close()
        # Optionally convert to MP3 (requires ffmpeg)
        subprocess.call(['ffmpeg', '-i', 'file.wav', 'file.mp3'])
        os.remove('file.wav')

def main(args=None):
    rclpy.init(args=args)
    node = AudioRecordingNode()
    rclpy.spin(node)
    node.stream.stop_stream()
    node.stream.close()
    node.pyaudio_instance.terminate()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
