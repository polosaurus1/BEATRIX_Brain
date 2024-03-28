import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from facenet_pytorch import MTCNN
import torch

class DualCameraPublisher(Node):
    def __init__(self):
        super().__init__('dual_camera_publisher')
        self.face_publisher = self.create_publisher(Image, 'camera1_face_image', 10)
        self.second_camera_publisher = self.create_publisher(Image, 'camera2_image', 10)  # Publisher for the second camera
        self.info_publisher = self.create_publisher(String, 'face_info', 10)
        self.bridge = CvBridge()

        self.cap1 = cv2.VideoCapture(0)  # First camera for face tracking
        self.cap2 = cv2.VideoCapture(2)  # Second camera, adjust the index as necessary
        
        # Check if cameras opened successfully
        if not self.cap1.isOpened() or not self.cap2.isOpened():
            self.get_logger().error('Could not open video streams')
            self.destroy_node()
            raise Exception('Could not open one or more video streams')

        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.mtcnn = MTCNN(keep_all=False, select_largest=True, device=self.device, min_face_size=40, factor=0.709, thresholds=[0.6, 0.7, 0.7])

        self.timer = self.create_timer(0.033, self.timer_callback)  # 30 FPS

    def timer_callback(self):
        ret1, frame1 = self.cap1.read()  # Read fr
        # om the first camera
        ret2, frame2 = self.cap2.read()  # Read from the second camera
        if ret1:
            # Convert the frame to RGBfdj-3079 and resize for detection
            frame_rgb = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
            frame_resized_for_detection = cv2.resize(frame_rgb, (160, 120))
            boxes, _ = self.mtcnn.detect(frame_resized_for_detection)

            if boxes is not None:
                # Calculate scaling factors
                scaling_factor_x = frame1.shape[1] / 160
                scaling_factor_y = frame1.shape[0] / 120

                for box in boxes:
                    # Scale box coordinates back to the original frame size
                    original_box = [
                        int(box[0] * scaling_factor_x),
                        int(box[1] * scaling_factor_y),
                        int((box[2] - box[0]) * scaling_factor_x),  # Width
                        int((box[3] - box[1]) * scaling_factor_y)  # Height
                    ]

                    # Draw rectangles around detected faces on the original frame
                    cv2.rectangle(frame1, (original_box[0], original_box[1]),
                                (original_box[0] + original_box[2], original_box[1] + original_box[3]), 
                                (0, 255, 0), 2)

                # Use the first box to calculate the face center and offsets
                x, y, w, h = original_box
                face_center_x = x + w // 2
                face_center_y = y + h // 2

                image_center_x = frame1.shape[1] // 2
                image_center_y = frame1.shape[0] // 2
                offset_x = face_center_x - image_center_x
                offset_y = face_center_y - image_center_y

                info_msg = f"Detected: True, Offset X: {offset_x}, Offset Y: {offset_y}"
                self.info_publisher.publish(String(data=info_msg))
            else:
                info_msg = "Detected: False"
                self.info_publisher.publish(String(data=info_msg))

            ros_image1 = self.bridge.cv2_to_imgmsg(frame1, 'bgr8')
            self.face_publisher.publish(ros_image1)

        # Simply publish the second camera's frame without any processing
        if ret2:
            ros_image2 = self.bridge.cv2_to_imgmsg(frame2, 'bgr8')
            self.second_camera_publisher.publish(ros_image2)  # Publish the frame from the second camera
        
        if not ret1 or not ret2:
            self.get_logger().error('Error capturing frames from cameras')


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.cap1.release()
        node.cap2.release()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()