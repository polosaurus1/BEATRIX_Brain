import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
import smbus
import time

# Configuration
bus = smbus.SMBus(1)
address = 0x48
A0 = 0x40  # Channel A0 command
output_file = "adc_data_command_clips.txt"
sample_rate = 10000  # Hz, adjust as needed
interval = 1.0 / sample_rate

# High-pass filter setup
cutoff_frequency = 70  # Hz, cutoff frequency for the high-pass filter
nyquist_frequency = sample_rate / 2.0
normalized_cutoff_frequency = cutoff_frequency / nyquist_frequency

# Design a Butterworth high-pass filter
b, a = butter(N=5, Wn=normalized_cutoff_frequency, btype='high', analog=False)

try:
    data_a0 = []  # Store ADC data
    print("Recording... Press CTRL+C to stop.")
    start_time = time.time()
    while True:
        # Read from A0
        bus.write_byte(address, A0)
        value0 = bus.read_byte(address)
        data_a0.append(value0)

        # Maintain sample rate
        time.sleep(interval)

except KeyboardInterrupt:
    # Filtering the collected data
    filtered_data_a0 = filtfilt(b, a, data_a0)
    
    # Saving the filtered data
    with open(output_file, "w") as f:
        current_time = start_time
        for value in filtered_data_a0:
            f.write(f"{current_time},{value}\n")
            current_time += interval

    print(f"Data recording stopped. Filtered data saved in {output_file}.")

    # Plotting
    time_axis = np.arange(0, len(data_a0) * interval, interval)
    plt.figure(figsize=(10, 6))
    plt.plot(time_axis, data_a0, label='Original Channel A0')
    plt.plot(time_axis, filtered_data_a0, label='Filtered Channel A0', color='red')
    plt.xlabel('Time (seconds)')
    plt.ylabel('ADC Value')
    plt.title('Filtered vs. Original Amplitude Variation Over Time for Channel A0')
    plt.legend()
    plt.grid(True)
    plt.show()
