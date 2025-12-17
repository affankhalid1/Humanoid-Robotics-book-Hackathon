---
sidebar_position: 2
---

# Chapter 1: OpenAI Whisper Integration

## Introduction

OpenAI Whisper is a state-of-the-art speech recognition model that converts spoken language into text. This chapter covers the integration of Whisper with robotic systems for voice command processing. Whisper provides robust speech-to-text capabilities that work across multiple languages and handle various acoustic conditions, making it ideal for robotic applications.

## Whisper Architecture

### Model Overview

Whisper is a large-scale multilingual speech recognition model:

- **Transformer-based**: Uses attention mechanisms for sequence processing
- **Multilingual**: Trained on 98 languages
- **Robust**: Handles various accents, background noise, and audio qualities
- **Flexible**: Available in multiple sizes (tiny, base, small, medium, large)

### Model Variants

| Model | Size | Required VRAM | Relative Speed |
|-------|------|---------------|----------------|
| tiny  | 75 MB | ~1 GB | ~32x |
| base  | 142 MB | ~1 GB | ~16x |
| small | 465 MB | ~2 GB | ~6x |
| medium | 1.5 GB | ~5 GB | ~2x |
| large | 3.0 GB | ~10 GB | 1x |

## Installation and Setup

### Python Installation

Install Whisper and related dependencies:

```bash
pip install openai-whisper
pip install torch torchvision torchaudio
pip install pyaudio  # For real-time audio capture
pip install soundfile  # For audio file processing
```

### System Requirements

- **CPU**: Modern processor with AVX support
- **GPU**: CUDA-capable GPU recommended for real-time processing
- **Memory**: 8GB+ RAM for large models
- **Storage**: 3GB+ for model files

## Basic Whisper Usage

### Transcription Examples

```python
import whisper

# Load model
model = whisper.load_model("base")

# Transcribe audio file
result = model.transcribe("audio.wav")
print(result["text"])

# Transcribe with language specification
result = model.transcribe("audio.wav", language="en")
print(result["text"])
```

### Real-time Audio Processing

```python
import whisper
import pyaudio
import numpy as np
import wave

class WhisperTranscriber:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio_format = pyaudio.paFloat32
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024

    def transcribe_audio(self, audio_data):
        # Convert audio to numpy array
        audio_array = np.frombuffer(audio_data, dtype=np.float32)

        # Transcribe
        result = self.model.transcribe(audio_array)
        return result["text"]

    def listen_and_transcribe(self, duration=5):
        p = pyaudio.PyAudio()

        stream = p.open(format=self.audio_format,
                        channels=self.channels,
                        rate=self.rate,
                        input=True,
                        frames_per_buffer=self.chunk)

        print("Recording...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)

        print("Transcribing...")
        audio_data = b''.join(frames)

        # Save to temporary file for Whisper
        with wave.open("temp.wav", 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(p.get_sample_size(self.audio_format))
            wf.setframerate(self.rate)
            wf.writeframes(audio_data)

        # Transcribe the audio
        result = self.model.transcribe("temp.wav")
        stream.stop_stream()
        stream.close()
        p.terminate()

        return result["text"]
```

## Whisper with ROS 2 Integration

### Creating a Whisper ROS Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import whisper
import numpy as np

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Load Whisper model
        self.model = whisper.load_model("base")

        # Create publisher for transcribed text
        self.text_pub = self.create_publisher(String, 'transcribed_text', 10)

        # Create subscriber for audio data
        self.audio_sub = self.create_subscription(
            AudioData, 'audio_input', self.audio_callback, 10)

        self.get_logger().info('Whisper node started')

    def audio_callback(self, msg):
        try:
            # Convert audio data to numpy array
            audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32)
            audio_array /= 32768.0  # Normalize to [-1, 1]

            # Transcribe audio
            result = self.model.transcribe(audio_array)

            # Publish transcribed text
            text_msg = String()
            text_msg.data = result["text"]
            self.text_pub.publish(text_msg)

            self.get_logger().info(f'Transcribed: {result["text"]}')

        except Exception as e:
            self.get_logger().error(f'Error in transcription: {e}')
```

### Audio Data Configuration

```yaml
# Audio configuration for Whisper integration
whisper_node:
  ros__parameters:
    model_size: "base"  # tiny, base, small, medium, large
    language: "en"      # Language code
    sample_rate: 16000  # Audio sample rate
    chunk_size: 1024    # Audio chunk size
    vad_enabled: true   # Voice activity detection
    vad_threshold: 0.3  # Voice activity threshold
```

## Advanced Whisper Features

### Voice Activity Detection

Implement voice activity detection to optimize processing:

```python
import webrtcvad
import collections

class VoiceActivityDetector:
    def __init__(self, sample_rate=16000, frame_duration=30):
        self.vad = webrtcvad.Vad(2)  # Aggressiveness level 2
        self.sample_rate = sample_rate
        self.frame_duration = frame_duration
        self.frame_size = int(sample_rate * frame_duration / 1000)

    def is_speech(self, audio_frame):
        return self.vad.is_speech(audio_frame, self.sample_rate)

    def detect_voice_activity(self, audio_data):
        frames = self.frame_generator(self.frame_duration, audio_data, self.sample_rate)
        voiced_frames = []

        for frame in frames:
            if self.is_speech(frame):
                voiced_frames.append(frame)

        return len(voiced_frames) > 0

    def frame_generator(self, frame_duration_ms, audio_data, sample_rate):
        n = int(sample_rate * (frame_duration_ms / 1000.0) * 2)
        offset = 0
        while offset + n < len(audio_data):
            yield audio_data[offset:offset + n]
            offset += n
```

### Speaker Diarization

Identify different speakers in audio:

```python
import pyannote.audio
from pyannote.audio import Pipeline

class SpeakerDiarization:
    def __init__(self, huggingface_token):
        self.pipeline = Pipeline.from_pretrained(
            "pyannote/speaker-diarization",
            use_auth_token=huggingface_token
        )

    def diarize_audio(self, audio_file):
        # Apply diarization
        diarization = self.pipeline(audio_file)

        # Group segments by speaker
        speakers = {}
        for turn, _, speaker in diarization.itertracks(yield_label=True):
            if speaker not in speakers:
                speakers[speaker] = []
            speakers[speaker].append((turn.start, turn.end))

        return speakers
```

## Performance Optimization

### GPU Acceleration

Use GPU for faster processing:

```python
import whisper
import torch

# Check if CUDA is available
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# Load model on GPU
model = whisper.load_model("base").to(device)

# Transcribe with GPU acceleration
result = model.transcribe("audio.wav")
```

### Batch Processing

Process multiple audio segments efficiently:

```python
def batch_transcribe(audio_segments, model):
    results = []
    for segment in audio_segments:
        result = model.transcribe(segment)
        results.append(result["text"])
    return results

# Or use Whisper's built-in batch processing
def efficient_batch_transcribe(audio_files, model):
    # Load all audio files
    audio_data = [whisper.load_audio(file) for file in audio_files]

    # Pad or trim to match model input
    audio_data = [whisper.pad_or_trim(audio) for audio in audio_data]

    # Convert to tensor
    mel = whisper.log_mel_spectrogram(audio_data).to(model.device)

    # Process in batch
    with torch.no_grad():
        result = model.decode(mel, whisper.DecodingOptions())

    return [segment.text for segment in result]
```

## Quality of Service Considerations

### Audio Quality Requirements

For optimal Whisper performance:

- **Sample Rate**: 16kHz preferred
- **Bit Depth**: 16-bit minimum
- **Audio Format**: WAV, MP3, or FLAC
- **Noise Level**: Minimize background noise
- **Volume**: Consistent and clear speech

### Latency Optimization

Minimize transcription latency:

```python
class OptimizedWhisperNode(Node):
    def __init__(self):
        super().__init__('optimized_whisper_node')

        # Pre-load model to avoid loading delay
        self.model = whisper.load_model("base").to("cuda" if torch.cuda.is_available() else "cpu")

        # Use threading for non-blocking processing
        self.processing_thread = None
        self.audio_queue = collections.deque(maxlen=10)

        # Create timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_audio_queue)

    def process_audio_queue(self):
        if self.audio_queue:
            audio_data = self.audio_queue.popleft()
            # Process in background thread to avoid blocking
            if not self.processing_thread or not self.processing_thread.is_alive():
                self.processing_thread = threading.Thread(
                    target=self.transcribe_and_publish,
                    args=(audio_data,)
                )
                self.processing_thread.start()
```

## Troubleshooting Common Issues

### Model Loading Issues

**Issue**: Memory errors with large models
**Solutions**:
1. Use smaller model variants (tiny, base)
2. Ensure sufficient GPU memory
3. Use CPU-only processing if GPU memory is limited

### Audio Format Problems

**Issue**: Unsupported audio formats
**Solutions**:
1. Convert audio to supported formats (WAV, MP3)
2. Ensure proper sample rate (16kHz)
3. Check audio bit depth and encoding

### Recognition Accuracy

**Issue**: Poor transcription accuracy
**Solutions**:
1. Improve audio quality and reduce noise
2. Use appropriate model size for accuracy needs
3. Specify language if known
4. Preprocess audio for noise reduction

## Best Practices

1. **Model Selection**: Choose model size based on accuracy and performance requirements
2. **Audio Quality**: Ensure high-quality audio input for best results
3. **Resource Management**: Monitor memory and processing usage
4. **Error Handling**: Implement robust error handling for audio processing
5. **Real-time Considerations**: Optimize for real-time processing requirements

## Integration with Robot Systems

### Voice Command Recognition

```python
class VoiceCommandRecognizer:
    def __init__(self):
        self.model = whisper.load_model("base")
        self.command_keywords = [
            "move forward", "turn left", "turn right", "stop",
            "go to", "find", "pick up", "drop", "follow me"
        ]

    def recognize_command(self, audio_input):
        transcription = self.model.transcribe(audio_input)["text"]

        # Identify command keywords in transcription
        recognized_commands = []
        for keyword in self.command_keywords:
            if keyword.lower() in transcription.lower():
                recognized_commands.append(keyword)

        return {
            "transcription": transcription,
            "commands": recognized_commands,
            "confidence": self.estimate_confidence(transcription)
        }

    def estimate_confidence(self, text):
        # Simple confidence estimation based on text quality
        # In practice, use more sophisticated methods
        return min(1.0, len(text) / 100.0)
```

## Summary

In this chapter, we covered:

- Whisper model architecture and variants
- Installation and basic usage
- ROS 2 integration approaches
- Performance optimization techniques
- Quality of service considerations
- Troubleshooting common issues

In the next chapter, we'll explore LangChain for task planning and natural language understanding.