import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import wave
import os
import datetime
import numpy as np

class BagToWavSaver(Node):
    def __init__(self):
        super().__init__('bag_to_wav_saver')
        self.num_mics = 8
        self.sample_rate = 44100
        self.save_folder = os.path.expanduser("~/ros2_ws/bag_audio_output")
        os.makedirs(self.save_folder, exist_ok=True)
        self.subscribers = []
        self.buffers = [[] for _ in range(self.num_mics)]

        for i in range(self.num_mics):
            topic = f"/mic{i+1}/audio"
            self.subscribers.append(
                self.create_subscription(Float32MultiArray, topic, self.make_callback(i), 10)
            )

    def make_callback(self, mic_index):
        def callback(msg):
            self.buffers[mic_index].extend(msg.data)
            if len(self.buffers[mic_index]) >= self.sample_rate * 1:  # Save every ~1 second of data
                filename = os.path.join(
                    self.save_folder,
                    f"mic{mic_index+1}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.wav"
                )
                data = np.array(self.buffers[mic_index], dtype=np.float32)
                with wave.open(filename, 'wb') as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)  # 16-bit PCM
                    wf.setframerate(self.sample_rate)
                    wf.writeframes((data * 32767).astype(np.int16).tobytes())
                self.get_logger().info(f"Saved: {filename}")
                self.buffers[mic_index] = []
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = BagToWavSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
