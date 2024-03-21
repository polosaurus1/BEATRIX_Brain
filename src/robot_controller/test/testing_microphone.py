import smbus
import time

# Configuration
bus = smbus.SMBus(1)
address = 0x48
A0 = 0x40  # Channel A0 command
output_file = "adc_data_command_clips.txt"
sample_rate = 10000  # Hz, adjust as needed
interval = 1.0 / sample_rate
noise_floor = (55, 75)  # Noise level to be filtered out

try:
    with open(output_file, "w") as f:
        print("Recording... Press CTRL+C to stop.")
        while True:
            # Read from A0
            bus.write_byte(address, A0)
            value0 = bus.read_byte(address)
            
            # Filter out specified noise levels
            if value0 < noise_floor[0] or value0 > noise_floor[1]:
                # Save timestamp and filtered A0 value
                f.write(f"{time.time()},{value0}\n")
            
            time.sleep(interval)  # Sleep to maintain the sample rate

except KeyboardInterrupt:
    # Save the data and close the file upon keyboard interrupt
    print(f"Data recording stopped. Data saved in {output_file}.")

# Optional: Comment out if you don't need immediate plotting after recording
import matplotlib.pyplot as plt

# Load the recorded data
timestamps, data_a0 = [], []
with open(output_file, "r") as file:
    for line in file:
        timestamp, value0 = line.strip().split(',')
        timestamps.append(float(timestamp))
        data_a0.append(int(value0))

# Assuming the first timestamp as the reference
time_axis = [t - timestamps[0] for t in timestamps]

# Plotting channel A0
plt.figure(figsize=(10, 6))
plt.plot(time_axis, data_a0, label='Channel A0')
plt.xlabel('Time (seconds)')
plt.ylabel('ADC Value')
plt.title('Filtered Amplitude Variation Over Time for Channel A0')
plt.legend()
plt.grid(True)
plt.show()
