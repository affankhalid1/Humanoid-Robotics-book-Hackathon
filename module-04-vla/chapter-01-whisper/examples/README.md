# Whisper Integration Example

This example demonstrates speech-to-text conversion using OpenAI Whisper for robotic applications.

## Overview

This example shows how to:
- Set up Whisper for speech recognition
- Process audio files and real-time audio
- Integrate with robotic command processing
- Optimize for real-time performance

## Prerequisites

- Python 3.10+
- OpenAI Whisper: `pip install openai-whisper`
- PyAudio: `pip install pyaudio`
- PyTorch: `pip install torch torchvision torchaudio`

## Usage

### Basic Transcription

```python
import whisper

# Load model
model = whisper.load_model("base")

# Transcribe audio file
result = model.transcribe("audio.wav")
print(result["text"])
```

### Real-time Audio Processing

```python
import whisper
import pyaudio
import numpy as np

# Initialize Whisper model
model = whisper.load_model("base")

# Set up audio capture
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paFloat32,
                channels=1,
                rate=16000,
                input=True,
                frames_per_buffer=1024)

print("Recording... Speak now.")
frames = []

# Capture audio for 5 seconds
for i in range(0, int(16000 / 1024 * 5)):
    data = stream.read(1024)
    frames.append(data)

print("Transcribing...")

# Combine and convert audio data
audio_data = b''.join(frames)
audio_array = np.frombuffer(audio_data, dtype=np.float32)

# Transcribe
result = model.transcribe(audio_array)
print(f"Transcribed: {result['text']}")

# Clean up
stream.stop_stream()
stream.close()
p.terminate()
```

## Model Selection

Choose the appropriate Whisper model based on your requirements:

- **tiny**: Fastest, least accurate (75 MB)
- **base**: Good balance (142 MB)
- **small**: Better accuracy (465 MB)
- **medium**: High accuracy (1.5 GB)
- **large**: Best accuracy (3.0 GB)

## Performance Tips

- Use GPU acceleration when available
- Choose the smallest model that meets accuracy requirements
- Pre-process audio to improve quality
- Use appropriate sample rates (16kHz recommended)