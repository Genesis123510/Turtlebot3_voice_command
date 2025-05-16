#!/home/jallanic/coqui-venv/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from TTS.api import TTS
import os
import tempfile
import subprocess

class CoquiSpeaker(Node):
    def __init__(self):
        super().__init__('coqui_speaker')
        self.subscription = self.create_subscription(
            String,
            'speak',
            self.listener_callback,
            10)
        self.subscription  # prevent unused var warning

        self.get_logger().info("Loading Coqui TTS model...")
        self.tts = TTS(model_name="tts_models/en/ljspeech/vits", progress_bar=False)
        self.get_logger().info("Coqui TTS model ready.")

    def listener_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Speaking: {text}")

        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp:
            self.tts.tts_to_file(text=text, file_path=tmp.name)
            subprocess.run(["aplay", tmp.name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            os.remove(tmp.name)

def main(args=None):
    rclpy.init(args=args)
    node = CoquiSpeaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
