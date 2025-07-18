#!/usr/bin/env python3
"""
bag_to_wav_saver.py
───────────────────
Subscribe to /mic1/audio … /mic8/audio (Float32, BEST_EFFORT)
and write 1-second 16-bit WAV files for each microphone.

1. Start:  ros2 bag play my_recording       # in terminal-A
2. Run:    python3 bag_to_wav_saver.py      # in terminal-B
3. Files:  ~/ros2_ws/bag_audio_output/mic1_YYYYmmdd_HHMMSS_xxxxxx.wav
"""

import os, datetime, wave, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

# -------- edit to match your setup --------------------------------------
NUM_MICS     = 8
SAMPLE_RATE  = 44_100          # 48_000 if you recorded at 48 kHz
CHUNK_SEC    = 1               # length of each WAV file
OUT_DIR      = os.path.expanduser('~/ros2_ws/bag_audio_output')
# ------------------------------------------------------------------------

BUFFER_SAMPLES = SAMPLE_RATE * CHUNK_SEC
BEST_EFFORT_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST)

class BagToWavSaver(Node):
    def __init__(self):
        super().__init__('bag_to_wav_saver')
        os.makedirs(OUT_DIR, exist_ok=True)
        self.buffers = [[] for _ in range(NUM_MICS)]

        for mic in range(NUM_MICS):
            topic = f'/mic{mic+1}/audio'
            self.create_subscription(Float32MultiArray,
                                     topic,
                                     self._make_cb(mic),
                                     BEST_EFFORT_QOS)

        self.get_logger().info(
            f'Waiting for /mic*/audio — writing {CHUNK_SEC}s WAV chunks to {OUT_DIR}')

    # ------------------------ helpers -----------------------------------
    def _make_cb(self, mic_idx):
        def cb(msg: Float32MultiArray):
            self.buffers[mic_idx].extend(msg.data)
            while len(self.buffers[mic_idx]) >= BUFFER_SAMPLES:
                self._flush(mic_idx)
        return cb

    def _flush(self, mic_idx):
        data = np.asarray(self.buffers[mic_idx][:BUFFER_SAMPLES], dtype=np.float32)
        del self.buffers[mic_idx][:BUFFER_SAMPLES]

        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        fn = os.path.join(OUT_DIR, f'mic{mic_idx+1}_{ts}.wav')

        with wave.open(fn, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)             # 16-bit PCM
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes((data * 32767).astype(np.int16).tobytes())

        self.get_logger().info(f'Saved {fn}')

    def destroy_node(self):
        for i, buf in enumerate(self.buffers):
            if buf:                         # flush leftovers
                self._flush(i)
        super().destroy_node()

# --------------------------- main ---------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = BagToWavSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
