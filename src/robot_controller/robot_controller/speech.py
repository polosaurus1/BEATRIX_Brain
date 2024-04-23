import rclpy
from rclpy.node import Node
import numpy as np
import sounddevice as sd
import speech_recognition as sr
from std_msgs.msg import String
import queue

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        self.recognizer = sr.Recognizer()
        self.publisher_ = self.create_publisher(String, 'recognized_speech', 10)
        self.fs = 44100  # Sample rate
        self.chunk_size = int(self.fs * 3)  # Record in 2-second chunks
        self.audio_queue = queue.Queue()

        # Stream callback function
        def callback(indata, frames, time, status):
            if status:
                print(status)
            self.audio_queue.put(indata.copy())

        # Open a new sounddevice stream
        self.stream = sd.InputStream(callback=callback, channels=1, samplerate=self.fs, blocksize=self.chunk_size)
        self.stream.start()

    def recognize_speech(self):
        try:
            # Check if there are items in the queue
            while not self.audio_queue.empty():
                audio_data = self.audio_queue.get()
                audio_data = np.int16(audio_data * np.iinfo(np.int16).max)  # Convert to int16 for recognizer
                audio = sr.AudioData(audio_data.tobytes(), self.fs, 2)
                text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'Recognized Speech: {text}')
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().info('Google Speech Recognition could not understand audio')
        except sr.RequestError as e:
            self.get_logger().info(f'Could not request results from Google Speech Recognition service; {e}')
        except Exception as e:
            self.get_logger().info(f'Error during recognition: {str(e)}')

    def stop(self):
        """Stops the audio stream."""
        self.stream.stop()
        self.stream.close()

def main(args=None):
    rclpy.init(args=args)
    speech_recognition_node = SpeechRecognitionNode()

    try:
        while rclpy.ok():
            speech_recognition_node.recognize_speech()
            rclpy.spin_once(speech_recognition_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('Program terminated by user.')
    finally:
        speech_recognition_node.stop()
        speech_recognition_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
