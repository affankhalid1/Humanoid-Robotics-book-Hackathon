# Complete VLA Pipeline Example

This example demonstrates a complete Voice-Language-Action (VLA) system that processes voice commands and executes robotic actions.

## Overview

This example shows how to:
- Integrate Whisper for speech recognition
- Use LangChain for natural language understanding
- Plan and execute robotic actions from voice commands
- Implement real-time voice processing
- Integrate with ROS 2 for robotic control

## Prerequisites

- Python 3.10+
- OpenAI Whisper: `pip install openai-whisper`
- LangChain: `pip install langchain langchain-openai`
- PyAudio: `pip install pyaudio`
- PyTorch: `pip install torch`
- ROS 2 Humble
- Microphone for voice input

## Setup

1. Install all required packages
2. Set up OpenAI API key in `.env` file
3. Ensure audio input is working

## Usage

### Basic VLA Pipeline

```python
from vla_pipeline import VLAPipeline

# Initialize the complete VLA pipeline
vla_pipeline = VLAPipeline(
    whisper_model_size="base",
    llm_model="gpt-3.5-turbo"
)

# Process audio file
result = vla_pipeline.execute_pipeline("command.wav")
print(f"Transcription: {result['transcription']}")
print(f"Planned Actions: {result['planned_actions']}")
```

### Real-time Voice Processing

```python
from real_time_vla import RealTimeVLAProcessor

# Initialize real-time processor
processor = RealTimeVLAProcessor(vla_pipeline)

# Start listening
processor.start_listening()

# Process commands continuously
try:
    while True:
        processor.process_queued_audio()
        time.sleep(0.1)
except KeyboardInterrupt:
    processor.stop_listening()
```

### ROS 2 Integration

The example includes a complete ROS 2 node that:

1. Subscribes to audio input topics
2. Processes voice commands through the VLA pipeline
3. Publishes planned actions to robot control topics
4. Reports execution results back to the system

Example launch file:

```xml
<launch>
  <!-- VLA node -->
  <node pkg="vla_ros" exec="vla_node" name="vla_node">
    <param name="whisper_model" value="base"/>
    <param name="llm_model" value="gpt-3.5-turbo"/>
  </node>

  <!-- Audio input node -->
  <node pkg="audio_capture" exec="audio_node" name="audio_input"/>

  <!-- Robot control node -->
  <node pkg="robot_control" exec="control_node" name="robot_control"/>
</launch>
```

## Performance Considerations

- Use GPU acceleration for Whisper when available
- Choose appropriate model sizes for your performance requirements
- Monitor resource usage during real-time processing
- Implement proper error handling and fallback mechanisms

## Testing

The example includes:
- Unit tests for individual components
- Integration tests for the complete pipeline
- Performance benchmarks
- Error recovery testing

Run tests with:
```bash
python -m pytest tests/
```