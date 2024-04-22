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

class AudioRecordingNode(Node):
    def __init__(self):
        super().__init__('AudioRecordingNode')
        self.subscription = self.create_subscription(String, "/recording_control", self.recording_control_callback, 10)
        self.transcript_publisher = self.create_publisher(String, '/transcript_topic', 10)
        self.location_publisher = self.create_publisher(String, '/location_topic', 10)
        self.is_recording = False
        self.frames = []
        model_size = "small.en"
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
        self.publish_transcription(temp)
        os.remove(temp)
        self.frames.clear()

    def get_command(self, transcript):
        building_names = {
            'EB1': ['Engineering Building 1', 'EB1', 'E B 1', 'EB 1', 'Evee 1', 'Evee 1' 'EV 1', 'EV1'],
            'EB2': ['Engineering Building 2', 'EB2', 'E B 2', 'EB 2', 'Evee 2', 'Evee 2' 'EV 2', 'EV2'],
            'EB3': ['Engineering Building 3', 'EB3', 'E B 3', 'EB 3', 'Evee 3', 'Evee 3', 'EV 3', 'EV3'],
            'Textiles': ['Textiles Building', 'Textile', 'Textiles', 'Textile Building', 'Textile\'s'],
            'Hunt': ['Hunt Library', 'Library', 'Hunt', 'Hunt\'s']
        }
    
        variations = [re.escape(variation) for name in building_names for variation in building_names[name]]
        pattern = r'\b(?:' + '|'.join(variations) + r')\b'
        
        # Search the transcript for the first occurrence of any building name
        match = re.search(pattern, transcript, re.IGNORECASE)
        if match:
            found_variation = match.group(0)
            for canonical, variations in building_names.items():
                if found_variation.lower() in map(str.lower, variations):
                    print(f"First mentioned building name found: {canonical}")
                    self.location_publisher.publish(String(data=canonical))

    def publish_transcription(self, audio_prompt):
        chunks, _ = self.model.transcribe(audio_prompt)
        transcript = "/n".join([chunk.text for chunk in chunks])
        self.transcript_publisher.publish(String(data=transcript))
        print(transcript)
        self.get_command(transcript)
        

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
