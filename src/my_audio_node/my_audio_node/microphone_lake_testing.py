#!/usr/bin/env python3
"""
8-mic VOICE-GATE recorder with energy threshold
────────────────────────────────────────────
* Publishes one topic per microphone:  /mic1/audio … /mic8/audio   (Float32MultiArray)
* Also publishes:                      /audio_frame  /mic_energy  /voice_detected …
* WebRTC-VAD gate; no ML model required.
* Skips VAD on low-energy blocks below `energy_threshold`.
"""

from __future__ import annotations
import time, threading
from queue import Queue

import numpy as np
import sounddevice as sd
import webrtcvad
from scipy.signal import resample_poly  # needed for 44.1 kHz support

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, Float32MultiArray, Bool


# ───────── helpers ─────────
def rms(x: np.ndarray) -> float:
    """Compute root-mean-square of a signal array."""
    return float(np.sqrt(np.mean(x ** 2) + 1e-12))


class VoiceGateNode(Node):
    """ROS2 node for real-time voice gating using 8-mic array and WebRTC-VAD."""
    NUM_MICS = 8

    def __init__(self) -> None:
        super().__init__("voice_gate_node")

        # ── parameters (defaults for 44.1 kHz; change on launch) ──
        self.declare_parameter("device_index", 10)   # PortAudio index for OCTA-CAPTURE
        self.declare_parameter("sample_rate", 44100)
        self.declare_parameter("chunk_size", 882)    # 20 ms @ 44.1 kHz
        self.declare_parameter("voice_off_delay", 0.8)
        # Energy threshold for gating (tune per environment, e.g. 0.10–0.30)
        self.declare_parameter("energy_threshold", 0.15)
        p = self.get_parameter
        self.device        = p("device_index").value
        self.sr            = p("sample_rate").value
        self.chunk         = p("chunk_size").value
        self.voff          = p("voice_off_delay").value
        self.energy_thresh = p("energy_threshold").value
        self.mic_positions = {
            0: (0, 0),
            1: (0.45, 0),
            2: (0.9, 0),
            3: (0, 0.03),
            4: (0.9, 0.595),
            5: (0.45, 0.595),
            6: (0, 0.595),
            7: (0.9, 0.03)
        }
        self.reference_mic = 0
        self.get_logger().info(f"Mic positions loaded: {self.mic_positions}")

        # publishers
        self.pub_mic   = [
            self.create_publisher(Float32MultiArray, f"/mic{i+1}/audio", 5)
            for i in range(self.NUM_MICS)
        ]
        self.pub_frame  = self.create_publisher(Float32MultiArray, "/audio_frame",  5)
        self.pub_energy = self.create_publisher(Float32MultiArray, "/mic_energy",   5)
        self.pub_detect = self.create_publisher(Bool,             "/voice_detected", 5)
        self.pub_conf   = self.create_publisher(Float32,          "/voice_confidence", 5)
        self.pub_alive  = self.create_publisher(Bool,             "/voice_gate_alive", 1)

        # VAD + worker queue
        self.vad = webrtcvad.Vad(1)
        self.q: "Queue[np.ndarray]" = Queue()
        threading.Thread(target=self._worker, daemon=True).start()

        # state
        self.voice_state     = False
        self.last_voice_time = 0.0

        # open audio stream
        self._open_stream(self.device)

        # heartbeat
        self.create_timer(1.0, lambda: self.pub_alive.publish(Bool(data=True)))

    def _open_stream(self, dev):
        try:
            self.stream = sd.InputStream(
                device     = dev,
                samplerate = self.sr,
                channels   = self.NUM_MICS,
                blocksize  = self.chunk,
                dtype      = "float32",
                latency    = 0.1,
                callback   = self._audio_cb,
            )
            self.stream.start()
            self.get_logger().info(
                f"Stream OK on {dev} @ {self.sr} Hz, chunk {self.chunk}, threshold {self.energy_thresh}"
            )
        except Exception as e:
            self.get_logger().error(f"Audio interface error: {e}")
            raise SystemExit("Check device/rate/channels settings.")

    def _audio_cb(self, indata, frames, _time, status):
        if status:
            self.get_logger().warn(f"PortAudio status: {status}")
        self.q.put(indata.copy())

    def _worker(self):
        slice_len = int(0.02 * self.sr)  # 20 ms window

        while rclpy.ok():
            block = self.q.get()

            # publish per-mic audio and raw frame
            for i in range(self.NUM_MICS):
                self.pub_mic[i].publish(
                    Float32MultiArray(data=block[:, i].tolist())
                )
            self.pub_frame.publish(
                Float32MultiArray(data=block.flatten().tolist())
            )

            # compute energies
            energies = [rms(block[:, i]) for i in range(self.NUM_MICS)]
            # print per-mic energies
            print("Mic RMS per channel:", energies)
            # publish energies
            self.pub_energy.publish(Float32MultiArray(data=energies))
            mean_energy = sum(energies) / len(energies)
            print(f"Mean RMS energy: {mean_energy:.6f}")

            # skip low-energy blocks
            if mean_energy < self.energy_thresh:
                continue

            # process 20 ms slices for VAD
            for start in range(0, len(block), slice_len):
                sub = block[start:start+slice_len, :]
                if len(sub) != slice_len:
                    continue
                mono = np.mean(sub, axis=1)
                # resample 44.1 → 16 kHz for VAD
                if self.sr == 44100:
                    data = resample_poly(mono, 16000, 44100)
                    rate = 16000
                else:
                    data = mono; rate = self.sr
                pcm16 = (data * 32767).astype(np.int16).tobytes()
                if not self.vad.is_speech(pcm16, sample_rate=rate):
                    continue
                # rising edge
                self.last_voice_time = time.time()
                if not self.voice_state:
                    self.voice_state = True
                    self.pub_detect.publish(Bool(data=True))
                # confidence
                conf = min(1.0, rms(mono) * 20)
                self.pub_conf.publish(Float32(data=conf))

            # falling edge: silence after delay
            if self.voice_state and time.time() - self.last_voice_time > self.voff:
                self.voice_state = False
                self.pub_detect.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = VoiceGateNode()
    exec_ = MultiThreadedExecutor(); exec_.add_node(node)
    threading.Thread(target=exec_.spin, daemon=True).start()
    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop(); node.stream.close()
        exec_.shutdown(); node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
