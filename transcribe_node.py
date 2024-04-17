import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import os
from pydub import AudioSegment

class transcribe_node(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.subscriber = self.create_subscription(String, 'audio_file_path', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, 'transcript', 10)
        self.model = whisper.load_model("base")

    def listener_callback(self, msg):
        self.get_logger().info('Received audio file path: "%s"' % msg.data)
        transcript = self.transcribe_audio(msg.data)
        self.publisher.publish(String(data=transcript))
        self.get_logger().info('Published transcript: "%s"' % transcript)

    def transcribe_audio(self, file_path):
        # Convert MP3 to WAV using pydub
        audio = AudioSegment.from_mp3(file_path)
        wav_path = file_path.replace(".mp3", ".wav")
        audio.export(wav_path, format="wav")

        # Load and transcribe using Whisper
        result = self.model.transcribe(wav_path)
        os.remove(wav_path)  # Clean up WAV file
        return result["text"]

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
