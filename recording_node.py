import pyaudio
import whisper
from faster_whisper import WhisperModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tempfile
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
        model_size = "base.en"
        model_str = f"ctranslate2-4you/whisper-{model_size}-ct2-int8"
        self.model = WhisperModel(model_str, device="cpu", compute_type="int8", cpu_threads=26)
        
    def recording_control_callback(self, msg):
        if msg.data == "start" and not self.is_recording:
            self.start_recording()
        elif msg.data == "stop" and self.is_recording:
            self.save_recording()
        
    def start_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.get_logger().info('Recording started...')
            threading.Thread(target=self.record_audio).start()

    def record_audio(self):
        self.audio = pyaudio.PyAudio()
        try: 
            self.stream = self.audio.open(format=pyaudio.paInt16, channels=1, rate=44100, input=True, frames_per_buffer=1024)
            while self.is_recording:
                self.frames.append(self.stream.read(1024))
            self.stream.stop_stream()
            self.stream.close()
        finally:
            self.audio.terminate()
            self.get_logger().info('Recording stopped.')

    
    def save_recording(self):
        self.is_recording = False
        temp = tempfile.mktemp(suffix=".wav")
        
        with wave.open(temp, "wb") as wave_file:
            wave_file.setnchannels(1)
            wave_file.setsampwidth(self.audio.get_sample_size(pyaudio.paInt16))
            wave_file.setframerate(44100)
            wave_file.writeframes(b"".join(self.frames))
            wave_file.close()
        self.transcribe_prompt(temp)
        os.remove(temp)
        self.frames.clear()

    def transcribe_prompt(self, audio_prompt):
        chunks, _ = self.model.transcribe(audio_prompt)
        transcript = "/n".join([chunk.text for chunk in chunks])
        print(transcript)

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
