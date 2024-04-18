import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
import speech_recognition as sr
import threading
import queue
from std_msgs.msg import String

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        self.audio_queue = queue.Queue()
        self.recognizer = sr.Recognizer()
        self.publisher_ = self.create_publisher(String, 'recognized_speech', 10)

        self.recording_thread = threading.Thread(target=self.record_audio_continuous)
        self.recording_thread.start()

        self.fs = 44100  # Sample rate
        self.listen_in_background()

    def record_audio_continuous(self):
        with sd.InputStream(channels=1, dtype='int16') as stream:
            while rclpy.ok():
                data, overflowed = stream.read(int(44100 * 20))
                if overflowed:
                    self.get_logger().info('Audio buffer has overflowed.')
                self.audio_queue.put(data)

    def callback(self, recognizer, audio):
        try:
            text = recognizer.recognize_google(audio)
            self.get_logger().info(f'Recognized Speech: {text}')
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().info('Google Speech Recognition could not understand audio')
        except sr.RequestError as e:
            self.get_logger().info(f'Could not request results from Google Speech Recognition service; {e}')

    def listen_in_background(self):
        self.recognizer.listen_in_background(sr.Microphone(), self.callback, phrase_time_limit=3)

def main(args=None):
    rclpy.init(args=args)
    speech_recognition_node = SpeechRecognitionNode()

    try:
        rclpy.spin(speech_recognition_node)
    except KeyboardInterrupt:
        print('Program terminated by user.')
    finally:
        speech_recognition_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
