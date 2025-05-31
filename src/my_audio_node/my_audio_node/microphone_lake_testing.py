import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
import wave
import os
from scipy.signal import butter, filtfilt
from std_msgs.msg import Float32MultiArray
import datetime
import csv

class AudioLocalizationNode(Node):
    def __init__(self):
        super().__init__('audio_localization_node')

        self.frame_counter = 0
        self.buffering = False
        self.buffer_count = 0
        self.BUFFER_CHUNKS = 100
        self.audio_buffer = []

        self.NUM_MICROPHONES = 8
        self.SAMPLE_RATE = 44100
        self.CHUNK_SIZE = 1024
        self.DEVICE_INDEX = 24
        self.THRESHOLD = 0.005
        self.SOUND_SPEED = 343
        self.SAVE_FOLDER = os.path.expanduser("~/ros2_ws/voice_detections")
        os.makedirs(self.SAVE_FOLDER, exist_ok=True)
        self.LOCATION_LOG = os.path.join(self.SAVE_FOLDER, "localization_log.csv")
        if not os.path.exists(self.LOCATION_LOG):
            with open(self.LOCATION_LOG, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Timestamp", "X (m)", "Y (m)"])

        self.board_length = 65.0
        self.board_width = 35.5

        self.mic_positions = {
            0: (0.0, 0.0),
            1: (33.0, 0.0),
            2: (65.0, 0.0),
            3: (0.0, 35.5),
            4: (33.0, 35.5),
            5: (65.0, 35.5),
            6: (0.0, 17.75),
            7: (65.0, 17.75),
        }

        self.reference_mic = 0
        self.mic_publishers = [
            self.create_publisher(Float32MultiArray, f'/mic{i+1}/audio', 10)
            for i in range(self.NUM_MICROPHONES)
        ]

        self.get_logger().info("Audio node initialized. Starting stream...")
        self.start_stream()

    def bandpass_filter(self, data, lowcut, highcut, fs, order=5):
        nyquist = 0.5 * fs
        low = lowcut / nyquist
        high = highcut / nyquist
        b, a = butter(order, [low, high], btype='band')
        return filtfilt(b, a, data)

    def gcc_phat(self, sig1, sig2, fs):
        n = len(sig1) + len(sig2)
        SIG1 = np.fft.rfft(sig1, n=n)
        SIG2 = np.fft.rfft(sig2, n=n)
        R = SIG1 * np.conj(SIG2)
        cc = np.fft.irfft(R / (np.abs(R) + 1e-15), n=n)
        max_shift = int(len(cc) / 2)
        shift = np.argmax(np.abs(cc)) - max_shift
        return shift / fs

    def estimate_position(self, tdoas, mic_positions, ref_index, sound_speed):
        ref_pos = np.array(mic_positions[ref_index])
        A, b = [], []
        for i, tdoa in tdoas.items():
            mic_pos = np.array(mic_positions[i])
            delta = mic_pos - ref_pos
            A.append(delta)
            b.append(0.5 * (np.linalg.norm(mic_pos)**2 - np.linalg.norm(ref_pos)**2 - (sound_speed * tdoa)**2))
        A = np.array(A)
        b = np.array(b)
        try:
            pseudo_inv = np.linalg.pinv(A)
            position = pseudo_inv @ b
            return position
        except:
            return ref_pos

    def callback(self, indata, frames, time, status):
        audio_data = np.array(indata)
        filtered_audio = np.zeros_like(audio_data)
        for i in range(self.NUM_MICROPHONES):
            filtered_audio[:, i] = self.bandpass_filter(audio_data[:, i], 300.0, 3400.0, self.SAMPLE_RATE)

        for i in range(self.NUM_MICROPHONES):
            mic_msg = Float32MultiArray()
            mic_msg.data = filtered_audio[:, i].tolist()
            self.mic_publishers[i].publish(mic_msg)
            

        ref_signal = filtered_audio[:, self.reference_mic]
        energy = np.sum(ref_signal**2)
        mic_energy = [np.sum(filtered_audio[:, i] ** 2) for i in range(self.NUM_MICROPHONES)]

        if energy < 0.01:
            self.get_logger().info("No voice detected.")
            return

        if self.buffering:
            self.audio_buffer.append(filtered_audio.copy())
            self.buffer_count -= 1
            if self.buffer_count == 0:
                full_audio = np.vstack(self.audio_buffer)
                tdoas = {}
                ref_signal = full_audio[:, self.reference_mic]
                for i in range(self.NUM_MICROPHONES):
                    if i == self.reference_mic:
                        continue
                    tdoa = self.gcc_phat(ref_signal, full_audio[:, i], self.SAMPLE_RATE)
                    tdoas[i] = tdoa
                source_pos = self.estimate_position(tdoas, self.mic_positions, self.reference_mic, self.SOUND_SPEED)
                with open(self.LOCATION_LOG, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([
                        datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        round(source_pos[0], 4),
                        round(source_pos[1], 4)
                    ])
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                for i in range(self.NUM_MICROPHONES):
                    voice_mic = full_audio[:, i]
                    mic_filename = os.path.join(
                        self.SAVE_FOLDER,
                        f"voice_{timestamp}_mic{i+1}.wav"
                    )
                    wf = wave.open(mic_filename, 'wb')
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(self.SAMPLE_RATE)
                    wf.writeframes((voice_mic * 32767).astype(np.int16).tobytes())
                    wf.close()
                    self.get_logger().info(f"Saved: {mic_filename}")
                self.buffering = False
                self.audio_buffer = []

        self.buffering = True
        self.buffer_count = self.BUFFER_CHUNKS
        self.audio_buffer = [filtered_audio.copy()]
        self.get_logger().info("Voice detected — starting buffer collection.")

    def start_stream(self):
        with sd.InputStream(
            channels=self.NUM_MICROPHONES,
            samplerate=self.SAMPLE_RATE,
            blocksize=self.CHUNK_SIZE,
            dtype='float32',
            device=self.DEVICE_INDEX,
            callback=self.callback
        ):
            self.get_logger().info("Streaming started... Press Ctrl+C to stop.")
            try:
                while rclpy.ok():
                    pass
            except KeyboardInterrupt:
                self.get_logger().info("Keyboard interrupt detected, stopping...")

def main(args=None):
    rclpy.init(args=args)
    node = AudioLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
