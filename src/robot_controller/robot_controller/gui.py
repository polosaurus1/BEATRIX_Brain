import signal
import os
import sys
import psutil
import subprocess
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PySide6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QPushButton, QHBoxLayout
from PySide6.QtGui import QImage, QPixmap, QCloseEvent
from PySide6.QtCore import Qt
import time

class CameraFeedViewer(Node):
    def __init__(self):
        super().__init__('camera_feed_viewer')
        self.cv_bridge = CvBridge()
        self.subscription = None  # Initialize without subscribing

    def subscribe_to_camera(self):
        if self.subscription is None:
            self.subscription = self.create_subscription(
                Image,
                'camera1_face_image',
                self.listener_callback,
                10)
            self.get_logger().info('Camera feed subscribed.')

    def unsubscribe_from_camera(self):
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.get_logger().info('Camera feed unsubscribed.')

    def listener_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.display_image(cv_image)

    def display_image(self, cv_image):
        # Convert the cv_image into a QImage
        h, w, ch = cv_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_BGR888)
        p = convert_to_Qt_format.scaled(640, 480, Qt.KeepAspectRatio)
        self.pixmap = QPixmap.fromImage(p)
        if hasattr(self, 'image_label'):
            self.image_label.setPixmap(self.pixmap)

class MainWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("ROS 2 Camera Feed Viewer")
        
        self.layout = QVBoxLayout(self)
        
        # Image label for displaying camera feed
        self.image_label = QLabel(self)
        self.layout.addWidget(self.image_label)
        
        # Initialize serial connection for robot commands
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)  # Wait for connection to establish
        
        # Motors enabled state
        self.motors_enabled = False
        
        # Add an "Enable Motors" button
        self.enable_motors_button = QPushButton("Enable Motors")
        self.enable_motors_button.clicked.connect(self.toggle_motors)
        self.layout.addWidget(self.enable_motors_button)
        
        # Directional buttons
        self.up_button = QPushButton("Up")
        self.down_button = QPushButton("Down")
        self.left_button = QPushButton("Left")
        self.right_button = QPushButton("Right")

        # Layout for directional buttons (considering intuitive placement)
        direction_layout = QVBoxLayout()  # Changed to vertical layout for intuitive arrangement
        horizontal_layout = QHBoxLayout()  # To place left and right buttons next to each other
        
        direction_layout.addWidget(self.up_button, 0, Qt.AlignCenter)  # Align Up button in the center
        horizontal_layout.addWidget(self.left_button)
        horizontal_layout.addWidget(self.right_button)
        direction_layout.addLayout(horizontal_layout)  # Add horizontal layout to the vertical layout
        direction_layout.addWidget(self.down_button, 0, Qt.AlignCenter)  # Align Down button in the center
        
        self.layout.addLayout(direction_layout)
        
        # Connect buttons to methods
        self.up_button.clicked.connect(lambda: self.send_movement_command(0, 100))
        self.down_button.clicked.connect(lambda: self.send_movement_command(0, -100))
        self.left_button.clicked.connect(lambda: self.send_movement_command(-100, 0))
        self.right_button.clicked.connect(lambda: self.send_movement_command(100, 0))

        # Toggle camera and face controller buttons
        self.toggle_camera_button = QPushButton("Start Camera")
        self.toggle_camera_button.clicked.connect(self.toggle_camera)
        self.layout.addWidget(self.toggle_camera_button)

        self.toggle_face_controller_button = QPushButton("Start Face Controller")
        self.toggle_face_controller_button.clicked.connect(self.toggle_face_controller)
        self.layout.addWidget(self.toggle_face_controller_button)

        self.camera_running = False
        self.face_controller_running = False
        self.camera_process = None
        self.face_controller_process = None

    def toggle_motors(self):
        """Toggle the motors on or off."""
        if self.motors_enabled:
            # Motors are currently enabled, so disable them
            self.node.get_logger().info('Disabling motors.')
            self.ser.write(b'@ENMOTORS OFF\r')
            self.enable_motors_button.setText("Enable Motors")
        else:
            # Motors are currently disabled, so enable them and send initial commands
            self.node.get_logger().info('Enabling motors.')
            self.ser.write(b'@ENMOTORS ON\r')
            time.sleep(1)
            self.ser.write(b'@CALNOW\r')
            time.sleep(1)
            self.enable_motors_button.setText("Disable Motors")
        
        self.motors_enabled = not self.motors_enabled

    def send_movement_command(self, movement_x, movement_y):
        """Construct and send movement command based on button pressed."""
        command = f"@MOVRALL {movement_x+movement_y} {movement_x} {movement_x-movement_y} 200 200 200\r".encode('utf-8')
        self.ser.write(command)
        time.sleep(1)  # Ensure command is processed before sending another


    def toggle_camera(self):
        if self.camera_running:
            if self.camera_process:
                # Send SIGTERM signal to the entire process group
                os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
                self.camera_process.wait()  # Wait for the process group to terminate
                self.camera_process = None
            self.node.unsubscribe_from_camera()
            self.toggle_camera_button.setText("Start Camera")  # Corrected attribute name
            self.camera_running = False
        else:
            # Start the camera node as a subprocess in a new process group
            self.camera_process = subprocess.Popen(["ros2", "run", "robot_controller", "camera_control"], preexec_fn=os.setsid)
            self.node.subscribe_to_camera()
            self.toggle_camera_button.setText("Stop Camera")  # Corrected attribute name
            self.camera_running = True

            
    def toggle_face_controller(self):
        if self.face_controller_running:
            self.stop_subprocess(self.face_controller_process)
            self.toggle_face_controller_button.setText("Start Face Controller")
            self.face_controller_running = False
        else:
            self.face_controller_process = self.start_subprocess(["ros2", "run", "robot_controller", "face_controller"])
            self.toggle_face_controller_button.setText("Stop Face Controller")
            self.face_controller_running = True

    def start_subprocess(self, command):
        # Start a ROS 2 node as a subprocess in a new process group
        return subprocess.Popen(command, preexec_fn=os.setsid)

    def stop_subprocess(self, process):
        if process:
            # Create a psutil Process object based on the PID of the subprocess
            parent = psutil.Process(process.pid)
            # Iterate over all child processes and terminate them
            for child in parent.children(recursive=True):
                child.terminate()
            parent.terminate()
            parent.wait()  # Wait for the termination to complete

    def closeEvent(self, event: QCloseEvent):
        # Handle the camera process termination
        if self.camera_running and self.camera_process:
            self.terminate_process(self.camera_process)
            self.camera_process = None

        # Handle the face controller process termination
        if self.face_controller_running and self.face_controller_process:
            self.terminate_process(self.face_controller_process)
            self.face_controller_process = None

        # Ensure ROS node cleanup is also performed
        self.node.destroy_node()
        rclpy.shutdown()

        event.accept()  # Accept the close event to close the application

    def terminate_process(self, process):
        """Terminates a process and all of its children using psutil."""
        try:
            parent = psutil.Process(process.pid)
            for child in parent.children(recursive=True):  # Terminate child processes
                child.terminate()
            parent.terminate()  # Terminate the parent process
            parent.wait()  # Wait for the termination to complete
        except psutil.NoSuchProcess:
            pass  # Process already terminated

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    node = CameraFeedViewer()
    rclpy.spin_once(node, timeout_sec=0)
    
    window = MainWindow(node)
    node.image_label = window.image_label
    window.show()
    
    def spin():
        rclpy.spin(node)
    
    from threading import Thread
    spin_thread = Thread(target=spin, daemon=True)
    spin_thread.start()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()