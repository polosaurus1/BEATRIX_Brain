import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import sounddevice as sd
from scipy.signal import correlate
import time  # Make sure time is imported correctly

fs = 44100  # Sample rate
chunk_duration = 2  # Chunk duration in seconds
chunk_samples = int(fs * chunk_duration)  # Number of samples per chunk
amplitude_threshold = 0.1  # Adjust this threshold based on your needs

class AudioDirectionPublisher(Node):
    def __init__(self):
        super().__init__('audio_direction_publisher')
        self.publisher = self.create_publisher(String, 'sound_direction', 10)

    def publish_direction(self, direction):
        msg = String()
        msg.data = direction
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {direction}")

def audio_callback(indata, frames, time_info, status, node):
    if status:
        print(status)

    rms_amplitude = np.sqrt(np.mean(indata**2))
    if rms_amplitude > amplitude_threshold:
        signal_left = indata[:, 0]
        signal_right = indata[:, 1]
        tdoa = calculate_tdoa(signal_left, signal_right, fs)

        if tdoa > 0:
            direction = "Sound coming from right"
        elif tdoa < 0:
            direction = "Sound coming from left"
        else:
            direction = "Sound is in front"
        
        node.publish_direction(direction)
        time.sleep(3)  # Correct usage of sleep
    else:
        print(f"Amplitude too low, skipping direction estimation. Amplitude: {rms_amplitude:.2f}")

def calculate_tdoa(signal1, signal2, fs):
    correlation = correlate(signal1, signal2, mode='full')
    lag = np.argmax(correlation) - (len(signal1) - 1)
    return lag / fs

def main(args=None):
    rclpy.init(args=args)
    audio_direction_publisher = AudioDirectionPublisher()
    
    # Adjust the stream callback to include the ROS 2 node
    stream_callback = lambda indata, frames, time_info, status: audio_callback(indata, frames, time_info, status, audio_direction_publisher)
    
    with sd.InputStream(channels=2, callback=stream_callback, samplerate=fs, blocksize=chunk_samples):
        print("Monitoring... Press Ctrl+C to stop")
        rclpy.spin(audio_direction_publisher)

    audio_direction_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
