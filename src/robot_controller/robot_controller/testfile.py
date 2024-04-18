import serial
import numpy as np
from collections import deque
import time  # Import time module for timing

# Configuration
serial_port = '/dev/ttyACM0'  # Adjust this to your Arduino's serial port
baud_rate = 115200
max_points = 200  # Maximum number of points for averaging
noise_threshold = 30  # Set the noise threshold
direction_threshold = 5  # Difference needed to determine direction

# Set up the serial connection
ser = serial.Serial(serial_port, baud_rate)

# Initialize deques to store the ADC values
adc_values_left = deque(maxlen=max_points)
adc_values_right = deque(maxlen=max_points)

# Counter for direction decision
direction_counter = 0

# Time tracking for 2-second interval outputs
last_output_time = time.time()

def high_pass_filter(values, alpha=0.9):
    filtered = np.zeros(len(values))
    for i in range(1, len(values)):
        filtered[i] = alpha * (filtered[i-1] + values[i] - values[i-1])
    return filtered

def process_data():
    global direction_counter, last_output_time
    while True:
        if ser.in_waiting:
            try:
                line_str = ser.readline().decode('utf-8').strip()
                if line_str.startswith("left:"):
                    adc_value_left = int(line_str.split(":")[1])
                    adc_values_left.append(adc_value_left)
                elif line_str.startswith("right:"):
                    adc_value_right = int(line_str.split(":")[1])
                    adc_values_right.append(adc_value_right)
                    
                if len(adc_values_left) == max_points and len(adc_values_right) == max_points:
                    filtered_left = high_pass_filter(list(adc_values_left))
                    filtered_right = high_pass_filter(list(adc_values_right))
                    avg_abs_left = np.mean(np.abs(filtered_left))
                    avg_abs_right = np.mean(np.abs(filtered_right))

                    if avg_abs_left - avg_abs_right > direction_threshold:
                        direction_counter -= 1  # Turn left
                    elif avg_abs_right - avg_abs_left > direction_threshold:
                        direction_counter += 1  # Turn right
                    
                    # Check if 2 seconds have passed since the last output
                    if time.time() - last_output_time >= 2:
                        print("Direction Counter:", direction_counter)
                        if direction_counter > 0:
                            print("Turn left.")
                        elif direction_counter < 0:
                            print("Turn right.")
                        else:
                            print("Direction undetermined.")
                        
                        direction_counter = 0  # Reset the direction counter
                        last_output_time = time.time()  # Update the time of the last output

            except ValueError:
                continue

# Start processing the serial data
process_data()
