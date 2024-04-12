import sounddevice as sd
import numpy as np
import wave
import speech_recognition as sr
import threading
import time
import io
import queue

def record_audio_continuous(queue, fs=44100, channels=2, dtype='int16', buffer_duration=2):
    """Continuously record audio from the microphone and put chunks into a queue with overlapping segments."""
    overlap_duration = 0.5  # 0.5 seconds of overlap
    frame_overlap = int(fs * overlap_duration)
    frame_buffer_duration = buffer_duration + overlap_duration
    frame_buffer_size = int(fs * frame_buffer_duration)  # total buffer size including overlap

    with sd.InputStream(samplerate=fs, channels=channels, dtype=dtype) as stream:
        # Initialize prev_data correctly based on number of channels
        prev_data = np.zeros((frame_overlap, channels), dtype=dtype)
        while True:
            data, overflowed = stream.read(frame_buffer_size)
            if overflowed:
                print("Audio buffer has overflowed.")
            
            # Combine previous data (overlap) with current data
            combined_data = np.concatenate((prev_data, data))
            queue.put(combined_data)

            # Store the last part of this buffer to overlap with the next one
            prev_data = data[-frame_overlap:]

def process_audio_from_queue(queue, recognizer, fs=44100, channels=2):
    """Process audio chunks from the queue and perform speech recognition."""
    while True:
        if not queue.empty():
            data = queue.get()
            audio_data = np.frombuffer(data, dtype=np.int16)
            with io.BytesIO() as buffer:
                with wave.open(buffer, 'wb') as wf:
                    wf.setnchannels(channels)
                    wf.setsampwidth(2)
                    wf.setframerate(fs)
                    wf.writeframes(audio_data.tobytes())
                buffer.seek(0)
                with sr.AudioFile(buffer) as source:
                    audio = recognizer.record(source)
                    try:
                        text = recognizer.recognize_google(audio)
                        print(f"Recognized Speech: {text}")
                    except sr.UnknownValueError:
                        print("Could not understand audio")
                    except sr.RequestError as e:
                        print(f"Could not request results from speech recognition service; {e}")
        time.sleep(1)  # Check the queue every second

def main():
    fs = 44100  # Sample rate
    audio_queue = queue.Queue()
    recognizer = sr.Recognizer()

    # Start the recording thread
    recording_thread = threading.Thread(target=record_audio_continuous, args=(audio_queue, fs))
    recording_thread.start()

    # Start the processing thread
    processing_thread = threading.Thread(target=process_audio_from_queue, args=(audio_queue, recognizer, fs))
    processing_thread.start()

    try:
        recording_thread.join()
        processing_thread.join()
    except KeyboardInterrupt:
        print("Program terminated by user.")

if __name__ == '__main__':
    main()
