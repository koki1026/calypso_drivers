import pyaudio
import numpy as np
import wave
from scipy.signal import butter, lfilter

# Configuration
NUM_MICROPHONES = 8
SAMPLE_RATE = 44100
CHUNK_SIZE = 1024
DEVICE_INDEX = 0  # Roland Octa-Capture device index
THRESHOLD = 0.05  # Noise threshold for voice detection
OUTPUT_FILENAME = "recorded_audio.wav"

def main():
    # Initialize PyAudio
    audio = pyaudio.PyAudio()

    # Open audio stream
    stream = audio.open(
        format=pyaudio.paFloat32,
        channels=NUM_MICROPHONES,
        rate=SAMPLE_RATE,
        input=True,
        input_device_index=DEVICE_INDEX,
        frames_per_buffer=CHUNK_SIZE
    )

    print("Listening for voice...")

    # Prepare to save audio
    frames = []

    try:
        while True:
            # Read audio data
            data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
            frames.append(data)  # Save raw audio data
            audio_data = np.frombuffer(data, dtype=np.float32)
            audio_data = audio_data.reshape(-1, NUM_MICROPHONES)

            # Process each microphone
            mic_energy = []
            for mic_index in range(NUM_MICROPHONES):
                mic_signal = audio_data[:, mic_index]
                mic_signal = mic_signal - np.mean(mic_signal)  # Remove DC offset
                mic_signal = mic_signal / np.max(np.abs(mic_signal))  # Normalize
                energy = np.sum(mic_signal ** 2)  # Calculate energy
                mic_energy.append(energy)

            # Detect voice and localize
            max_energy = max(mic_energy)
            if max_energy > THRESHOLD:
                active_mic = mic_energy.index(max_energy)
                print(f"Voice detected on microphone {active_mic + 1} with energy {max_energy:.2f}")
            else:
                print("No significant voice detected.")

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        # Save recorded audio to a .wav file
        print(f"Saving recorded audio to {recorded_audio.wav}...")
        wf = wave.open("recorded_datasets.wav", 'wb')

        # Apply Time Difference of Arrival (TDOA) for localization
        def calculate_tdoa(mic_signals, sample_rate):
            # Cross-correlate signals to find time delays
            delays = []
            reference_signal = mic_signals[0]  # Use the first microphone as reference
            for mic_signal in mic_signals[1:]:
                correlation = np.correlate(reference_signal, mic_signal, mode='full')
                delay = np.argmax(correlation) - (len(correlation) // 2)
                delays.append(delay / sample_rate)  # Convert delay to seconds
                return delays

        # Extract signals from filtered audio data
        mic_signals = [filtered_audio_data[:, mic_index] for mic_index in range(NUM_MICROPHONES)]

        # Calculate TDOA
        tdoa_delays = calculate_tdoa(mic_signals, SAMPLE_RATE)
        print(f"TDOA Delays (in seconds): {tdoa_delays}")
        wf.setnchannels(NUM_MICROPHONES)
        wf.setsampwidth(audio.get_sample_size(pyaudio.paFloat32))
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(b''.join(frames))
        wf.close()

        # Clean up
        stream.stop_stream()
        stream.close()
        audio.terminate()

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    def plot_localization(mic_energy):
        plt.bar(range(1, NUM_MICROPHONES + 1), mic_energy)
        plt.xlabel("Microphone Index")
        plt.ylabel("Energy")
        plt.title("Microphone Energy Levels")
        plt.show()

    mic_energy = []

    def main_with_plot():
        # Initialize PyAudio
        audio = pyaudio.PyAudio()

        # Open audio stream
        stream = audio.open(
            format=pyaudio.paFloat32,
            channels=NUM_MICROPHONES,
            rate=SAMPLE_RATE,
            input=True,
            input_device_index=DEVICE_INDEX,
            frames_per_buffer=CHUNK_SIZE
        )

        print("Listening for voice...")

        # Prepare to save audio
        frames = []

        try:
            while True:
                # Read audio data
                data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                frames.append(data)  # Save raw audio data
                audio_data = np.frombuffer(data, dtype=np.float32)
                audio_data = audio_data.reshape(-1, NUM_MICROPHONES)

                # Process each microphone
                mic_energy.clear()
                for mic_index in range(NUM_MICROPHONES):
                    mic_signal = audio_data[:, mic_index]
                    mic_signal = mic_signal - np.mean(mic_signal)  # Remove DC offset
                    mic_signal = mic_signal / np.max(np.abs(mic_signal))  # Normalize
                    energy = np.sum(mic_signal ** 2)  # Calculate energy
                    mic_energy.append(energy)

                # Detect voice and localize
                max_energy = max(mic_energy)
                if max_energy > THRESHOLD:
                    active_mic = mic_energy.index(max_energy)
                    print(f"Voice detected on microphone {active_mic + 1} with energy {max_energy:.2f}")
                else:
                    print("No significant voice detected.")

                # Plot localization with red dot
                plt.clf()
                plt.bar(range(1, NUM_MICROPHONES + 1), mic_energy, color='blue')
                if max_energy > THRESHOLD:
                    plt.scatter(active_mic + 1, max_energy, color='red', zorder=5, label="Active Mic")
                plt.xlabel("Microphone Index")
                plt.ylabel("Energy")
                plt.title("Microphone Energy Levels")
                plt.legend()
                plt.pause(0.01)

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            # Save recorded audio to a .wav file
            print(f"Saving recorded audio to {OUTPUT_FILENAME}...")
            wf = wave.open(OUTPUT_FILENAME, 'wb')
            wf.setnchannels(NUM_MICROPHONES)
            wf.setsampwidth(audio.get_sample_size(pyaudio.paFloat32))
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(b''.join(frames))
            wf.close()
            # Apply a band-pass filter to remove noise
            def bandpass_filter(data, lowcut, highcut, fs, order=5):
                nyquist = 0.5 * fs
                low = lowcut / nyquist
                high = highcut / nyquist
                b, a = butter(order, [low, high], btype='band')
                y = lfilter(b, a, data)
                return y

            # Filter the audio data for each microphone
            lowcut = 300.0  # Lower cutoff frequency in Hz
            highcut = 3400.0  # Upper cutoff frequency in Hz
            filtered_audio_data = np.zeros_like(audio_data)

            for mic_index in range(NUM_MICROPHONES):
                mic_signal = audio_data[:, mic_index]
                filtered_signal = bandpass_filter(mic_signal, lowcut, highcut, SAMPLE_RATE)
                filtered_audio_data[:, mic_index] = filtered_signal

            # Only process microphones 1 and 4
            mic_energy.clear()
            for mic_index in [0, 3]:  # Mic 1 is index 0, Mic 4 is index 3
                mic_signal = filtered_audio_data[:, mic_index]
                mic_signal = mic_signal - np.mean(mic_signal)  # Remove DC offset
                mic_signal = mic_signal / np.max(np.abs(mic_signal))  # Normalize
                energy = np.sum(mic_signal ** 2)  # Calculate energy
                mic_energy.append(energy)

            # Replace audio_data with filtered_audio_data for further processing
            audio_data = filtered_audio_data

            def bandpass_filter(data, lowcut, highcut, fs, order=5):
                nyquist = 0.5 * fs
                low = lowcut / nyquist
                high = highcut / nyquist
                b, a = butter(order, [low, high], btype='band')
                y = lfilter(b, a, data)
                return y

            # Filter the audio data for each microphone
            lowcut = 300.0  # Lower cutoff frequency in Hz
            highcut = 3400.0  # Upper cutoff frequency in Hz
            filtered_audio_data = np.zeros_like(audio_data)

            for mic_index in range(NUM_MICROPHONES):
                mic_signal = audio_data[:, mic_index]
                filtered_signal = bandpass_filter(mic_signal, lowcut, highcut, SAMPLE_RATE)
                filtered_audio_data[:, mic_index] = filtered_signal

            # Replace audio_data with filtered_audio_data for further processing
            audio_data = filtered_audio_data
            # Clean up
            stream.stop_stream()
            stream.close()
            audio.terminate()

    plt.ion()  # Enable interactive mode for live plotting
    main_with_plot()