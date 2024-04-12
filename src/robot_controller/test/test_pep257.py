import pyaudio
import wave

# Define the duration of the recording
record_seconds = 5  # for example, 5 seconds

# Initialize PyAudio
p = pyaudio.PyAudio()

# Audio recording parameters
format = pyaudio.paInt16  # 16 bits per sample
channels = 1  # Mono audio
sample_rate = 44100  # Sampling rate
chunk = 1024  # Frames per buffer
device_index = 1  # Assuming the device index is 1, adjust as necessary

# Open the audio stream
stream = p.open(format=format,
                channels=channels,
                rate=sample_rate,
                input=True,
                frames_per_buffer=chunk,
                input_device_index=device_index)

print(f"Recording for {record_seconds} seconds...")

frames = []

# Record for the set duration
for _ in range(0, int(sample_rate / chunk * record_seconds)):
    data = stream.read(chunk)
    frames.append(data)

# Stop and close the stream
stream.stop_stream()
stream.close()
# Terminate the PyAudio instance
p.terminate()

# Save the recorded data to a WAV file
output_filename = "output.wav"
wf = wave.open(output_filename, 'wb')
wf.setnchannels(channels)
wf.setsampwidth(p.get_sample_size(format))
wf.setframerate(sample_rate)
wf.writeframes(b''.join(frames))
wf.close()

print(f"Recording finished. The audio was saved to {output_filename}")
