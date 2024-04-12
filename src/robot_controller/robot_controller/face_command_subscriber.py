#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class FaceCommandSubscriber(Node):
    def __init__(self):
        super().__init__('face_command_subscriber')
        self.subscription = self.create_subscription(
            String,
            'face_info',  # Topic for combined face detection and offset information
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize serial connection
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)  # Wait for connection to establish

        # Send initial commands to Arduino
        self.send_initial_commands()

    def send_initial_commands(self):
        self.get_logger().info('Sending initial commands to Arduino.')
        self.ser.write(b'@ENMOTORS ON\r')
        time.sleep(1)
        self.ser.write(b'@CALNOW\r')
        time.sleep(1)

    def listener_callback(self, msg):
        # Parse the received message
        detected = "Detected: True" in msg.data
        if detected:
            # Extract X offset from the message
            parts = msg.data.split(',')
            offset_x = int(parts[1].split(':')[1].strip())  # Assuming this is the correct position for X offset
            offset_y = int(parts[2].split(':')[1].strip())  # Assuming this follows the X offset in the message
            
            # Calculate movement based on offset
            movement_x = -offset_x
            movement_y = -offset_y
            
            # Prepare command with calculated movement
            command = f'@MOVRALL {movement_x-movement_y} {int(movement_x+(0.5)* movement_y)} {movement_x} 200 200 200\r'.encode('utf-8')
            
            self.get_logger().info(f'Sending command to Arduino: {command.decode()}')
            self.ser.write(command)
            time.sleep(1)  # Delay before sending the next command
        else:
            self.get_logger().info('No face detected. No command sent.')


def main(args=None):
    rclpy.init(args=args)
    face_command_subscriber = FaceCommandSubscriber()
    try:
        rclpy.spin(face_command_subscriber)
    except:
        face_command_subscriber.ser.write(b'@ENMOTORS OFF\r')
        
        # Clean up
        face_command_subscriber.ser.close()
        face_command_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
