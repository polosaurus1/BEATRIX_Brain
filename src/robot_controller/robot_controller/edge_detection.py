    #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DualCamEdgeDetectionPublisher(Node):
    def __init__(self):
        super().__init__('dual_cam_edge_detection_publisher')
        # Publishers for each camera
        self.publisher1 = self.create_publisher(Image, 'camera1_edge_detected_image', 10)
        self.publisher2 = self.create_publisher(Image, 'camera2_edge_detected_image', 10)
        self.bridge = CvBridge()
        
        # Initialise video capture for both cameras
        self.cap1 = cv2.VideoCapture(0)  # First camera
        self.cap2 = cv2.VideoCapture(2)  # Second camera
        if not self.cap1.isOpened() or not self.cap2.isOpened():
            self.get_logger().error('Could not open one or both video streams')
            self.destroy_node()
            raise Exception('Could not open one or both video streams')
        
        # Timer for capturing and publishing frames from both cameras
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)  # 30 Hz

    def timer_callback(self):
        # Process and publish images from the first camera
        ret1, frame1 = self.cap1.read()
        if ret1:
            edges1 = cv2.Canny(cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY), 15, 150)
            ros_image1 = self.bridge.cv2_to_imgmsg(edges1, 'mono8')
            self.publisher1.publish(ros_image1)
        else:
            self.get_logger().error('Error capturing frame from camera 1')

        # Process and publish images from the second camera
        ret2, frame2 = self.cap2.read()
        if ret2:
            edges2 = cv2.Canny(cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY), 15, 150)
            ros_image2 = self.bridge.cv2_to_imgmsg(edges2, 'mono8')
            self.publisher2.publish(ros_image2)
        else:
            self.get_logger().error('Error capturing frame from camera 2')

def main(args=None):
    rclpy.init(args=args)
    node = DualCamEdgeDetectionPublisher()
    rclpy.spin(node)
    # Release resources for both cameras
    node.cap1.release()
    node.cap2.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
