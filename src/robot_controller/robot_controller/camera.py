#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Use String to publish detection status and offsets
from cv_bridge import CvBridge
import cv2

class FacePositionPublisher(Node):
    def __init__(self):
        super().__init__('face_position_publisher')
        # Publisher for the camera's detected faces
        self.face_publisher = self.create_publisher(Image, 'camera_face_image', 10)
        # Publisher for face detection status and relative position (offsets) of the detected face
        self.info_publisher = self.create_publisher(String, 'face_info', 10)  # Changed topic name to 'face_info'
        self.bridge = CvBridge()

        # Initialise video capture for the camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video stream')
            self.destroy_node()
            raise Exception('Could not open video stream')

        # Haar Cascade Classifier for face detection
        self.haar_cascade = cv2.CascadeClassifier("/home/ws/src/robot_controller/robot_controller/haarcascade_frontalface_default.xml")

        # Timer for capturing and processing frames from the camera
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert frame to grayscale for face detection
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect faces in the frame
            faces = self.haar_cascade.detectMultiScale(gray_frame, 1.1, 4)

            if len(faces) > 0:
                # Process the first detected face for simplicity
                x, y, w, h = faces[0]
                face_center_x = x + w // 2
                face_center_y = y + h // 2

                # Calculate offset from the center of the image
                image_center_x = frame.shape[1] // 2
                image_center_y = frame.shape[0] // 2
                offset_x = face_center_x - image_center_x
                offset_y = face_center_y - image_center_y

                # Construct and publish the combined message
                info_msg = f"Detected: True, Offset X: {offset_x}, Offset Y: {offset_y}"
                self.info_publisher.publish(String(data=info_msg))
                self.get_logger().info(f'Published face info: {info_msg}')
            else:
                # Publish message indicating no face is detected
                self.info_publisher.publish(String(data="Detected: False"))

            # Convert the frame to a ROS message and publish it
            ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.face_publisher.publish(ros_image)

        else:
            self.get_logger().error('Error capturing frame from camera')

def main(args=None):
    rclpy.init(args=args)
    node = FacePositionPublisher()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
