# Module 4: VLA Systems Troubleshooting Guide

## Installation Issues

### Whisper Installation Problems

**Issue**: Whisper fails to install or import
**Solutions**:
```bash
# Ensure proper Python version
python3 --version  # Should be 3.8+

# Install with proper dependencies
pip install --upgrade pip
pip install openai-whisper

# If CUDA issues occur
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

**Issue**: Model loading fails with memory error
**Solutions**:
1. Use smaller model variants (tiny, base)
2. Ensure sufficient RAM (8GB+ for large models)
3. Use CPU-only processing: `whisper.load_model("base", device="cpu")`

### LangChain Installation Issues

**Issue**: LangChain packages not found
**Solutions**:
```bash
pip install langchain langchain-openai langchain-community

# Check for version compatibility
pip list | grep langchain
```

**Issue**: API key authentication failures
**Solutions**:
1. Verify API key is set in environment
2. Check `.env` file format
3. Restart Python session after setting environment variables

## Audio Processing Issues

### Microphone Access Problems

**Issue**: No audio input detected
**Solutions**:
1. Check microphone permissions and connections
2. Verify audio device: `arecord -l`
3. Test with simple audio recording: `arecord -d 5 test.wav`

**Issue**: Poor audio quality for transcription
**Solutions**:
1. Use 16kHz sample rate for optimal Whisper performance
2. Reduce background noise during recording
3. Normalize audio levels before processing

### Real-time Processing Issues

**Issue**: High latency in voice processing
**Solutions**:
1. Use smaller Whisper models for faster processing
2. Optimize audio buffer sizes
3. Use GPU acceleration when available

## Language Model Issues

### LLM Connection Problems

**Issue**: API connection timeouts
**Solutions**:
1. Check internet connectivity
2. Verify API key validity
3. Implement retry logic with exponential backoff

**Issue**: High API costs
**Solutions**:
1. Use smaller models when possible (gpt-3.5-turbo vs gpt-4)
2. Implement caching for repeated requests
3. Optimize prompt lengths to reduce token usage

### Prompt Engineering Issues

**Issue**: Inconsistent command interpretations
**Solutions**:
1. Refine system prompts with clear instructions
2. Use structured output parsing
3. Implement prompt validation and testing

## ROS 2 Integration Issues

### Node Communication Problems

**Issue**: VLA node not receiving audio data
**Solutions**:
1. Verify topic names and message types
2. Check ROS domain ID consistency
3. Ensure proper network configuration

**Issue**: Robot action execution failures
**Solutions**:
1. Verify robot controller nodes are running
2. Check action server availability
3. Validate action message formats

### Threading and Concurrency Issues

**Issue**: Race conditions in VLA pipeline
**Solutions**:
1. Use proper thread synchronization
2. Implement queue-based processing
3. Use ROS 2's built-in thread safety features

## Performance Issues

### Processing Speed Problems

**Issue**: Slow transcription or planning
**Solutions**:
1. Use GPU acceleration for Whisper
2. Optimize LangChain chain composition
3. Implement caching for common commands
4. Use async processing where possible

**Issue**: High memory usage
**Solutions**:
1. Use smaller models when accuracy allows
2. Implement proper memory cleanup
3. Monitor and optimize data structures

### Real-time Performance

**Issue**: Missed voice commands due to processing delays
**Solutions**:
1. Implement priority queues for audio processing
2. Use separate threads for audio capture and processing
3. Optimize pipeline for minimal latency

## Quality of Service Issues

### Command Recognition Problems

**Issue**: Poor voice command recognition
**Solutions**:
1. Improve audio quality and reduce noise
2. Use wake word detection to trigger processing
3. Implement confidence thresholds for command acceptance

**Issue**: Misinterpreted commands
**Solutions**:
1. Add confirmation steps for critical commands
2. Implement context-aware command validation
3. Use structured command formats when needed

### Error Handling Issues

**Issue**: Unhandled exceptions causing system crashes
**Solutions**:
1. Implement comprehensive error handling
2. Add fallback mechanisms for critical failures
3. Use circuit breakers for external API calls

## Common Error Messages

### "CUDA out of memory"
- Use smaller models or CPU processing
- Clear GPU memory before loading models
- Monitor memory usage during processing

### "API rate limit exceeded"
- Implement request throttling
- Use appropriate retry mechanisms
- Consider higher rate limit plans if needed

### "Audio device not found"
- Check audio device permissions
- Verify device availability with `arecord -l`
- Ensure proper device configuration

## Debugging Tools

### Audio Debugging

```python
# Test audio input
import pyaudio
import numpy as np

p = pyaudio.PyAudio()
print("Available audio devices:")
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    print(f"Device {i}: {info['name']}")
```

### API Debugging

Enable LangChain debugging:
```python
from langchain.globals import set_debug
set_debug(True)
```

### Performance Profiling

Use Python profilers for performance analysis:
```bash
python -m cProfile -s cumulative your_vla_script.py
```

## Best Practices for Troubleshooting

1. **Start Simple**: Test basic components before complex integrations
2. **Isolate Issues**: Test Whisper and LangChain separately
3. **Monitor Resources**: Keep track of memory and CPU usage
4. **Log Everything**: Implement comprehensive logging
5. **Version Control**: Pin specific versions for reproducible environments

## Getting Help

If issues persist:

1. Check OpenAI Whisper documentation
2. LangChain official documentation and community
3. ROS 2 troubleshooting guides
4. Audio processing libraries documentation
5. Community forums and Q&A sites