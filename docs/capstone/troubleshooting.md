# Capstone Project: Troubleshooting Guide

## System Integration Issues

### ROS 2 Communication Problems

**Issue**: Nodes from different modules cannot communicate
**Solutions**:
1. Verify ROS_DOMAIN_ID is consistent across all containers
2. Check network configuration (using `network_mode: host`)
3. Ensure all nodes are on the same ROS network
4. Verify topic names and message types match

```bash
# Check ROS 2 network
ros2 topic list
ros2 node list
ros2 doctor
```

### Isaac ROS and Perception Issues

**Issue**: Perception pipeline not processing data correctly
**Solutions**:
1. Verify Isaac ROS packages are installed: `ros2 pkg list | grep isaac`
2. Check GPU access: `nvidia-smi`
3. Verify camera calibration and data flow
4. Check cuvSLAM initialization

### VLA System Integration Problems

**Issue**: Voice commands not processed through integrated system
**Solutions**:
1. Verify OpenAI API key is set in environment
2. Check Whisper model loading: `python -c "import whisper; whisper.load_model('tiny')"`
3. Verify LangChain configuration and API access
4. Check audio input device access

## Performance Issues

### System Resource Problems

**Issue**: High CPU or GPU usage causing performance degradation
**Solutions**:
1. Monitor resource usage: `htop`, `nvidia-smi`
2. Reduce processing rates in configuration files
3. Use smaller models for less critical tasks
4. Optimize data flow between modules

**Issue**: Memory exhaustion with integrated system
**Solutions**:
1. Implement proper memory management
2. Use memory pooling for large data structures
3. Monitor and clear unused data regularly
4. Consider using smaller model variants

### Real-time Performance

**Issue**: Latency between voice command and action execution
**Solutions**:
1. Optimize pipeline for minimal latency
2. Use async processing where appropriate
3. Reduce unnecessary data copying
4. Profile and optimize bottlenecks

## Hardware Integration Issues

### Sensor Data Problems

**Issue**: Camera feed not available to perception system
**Solutions**:
1. Check camera device permissions and connectivity
2. Verify camera driver and ROS 2 interface
3. Test camera independently before integration
4. Check camera calibration files

**Issue**: Audio input not working for VLA system
**Solutions**:
1. Verify microphone access and permissions
2. Test audio with simple recording: `arecord -d 5 test.wav`
3. Check audio device configuration in Docker
4. Verify PyAudio installation and configuration

### Robot Control Issues

**Issue**: Robot commands not being executed
**Solutions**:
1. Check robot controller availability
2. Verify command message formats
3. Test robot control independently
4. Check safety systems and emergency stops

## Common Error Messages

### "Failed to initialize Isaac ROS component"
- Check GPU driver compatibility
- Verify Isaac ROS installation
- Ensure sufficient GPU memory
- Check CUDA version compatibility

### "Voice command timeout"
- Verify audio input is working
- Check Whisper model loading
- Verify LangChain API access
- Check system performance

### "Navigation failed to plan path"
- Check map availability and quality
- Verify sensor data (LiDAR/camera) availability
- Check obstacle detection system
- Validate costmap configuration

## Debugging Strategies

### Component Isolation

Test each module independently before integration:

```bash
# Test ROS 2 fundamentals
ros2 topic echo /test_topic

# Test perception pipeline
ros2 run isaac_ros_apriltag apriltag_node

# Test VLA system
python3 -c "import whisper; print('Whisper OK')"

# Test navigation
ros2 action send_goal /navigate_to_pose ...
```

### Logging and Monitoring

Enable detailed logging for troubleshooting:

```bash
# Set ROS 2 logging level
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Monitor system performance
nvidia-smi -l 1
htop
iotop
```

### Integration Testing

Create simple integration tests:

```bash
# Test basic communication between modules
ros2 topic pub /voice_command std_msgs/String "data: 'test'"

# Monitor system status
ros2 topic echo /system_status

# Check all required nodes are running
ros2 node list | grep -E "(vla|perception|navigation|control)"
```

## Docker and Container Issues

### Container Communication

**Issue**: Services in different containers cannot communicate
**Solutions**:
1. Use `network_mode: host` in docker-compose
2. Verify shared volumes are properly mounted
3. Check environment variables are properly passed
4. Ensure consistent ROS_DOMAIN_ID across containers

### GPU Access in Containers

**Issue**: GPU acceleration not working in Docker
**Solutions**:
1. Install nvidia-docker2: `sudo apt install nvidia-docker2`
2. Restart Docker: `sudo systemctl restart docker`
3. Verify runtime: `docker info | grep -i nvidia`
4. Test GPU in container: `nvidia-smi`

## Performance Optimization

### Pipeline Optimization

1. **Reduce unnecessary processing**: Only process data when needed
2. **Optimize data formats**: Use efficient data representations
3. **Threading**: Use appropriate threading for I/O operations
4. **Caching**: Cache results of expensive operations when appropriate

### Resource Management

1. **Monitor usage**: Regularly check CPU, GPU, and memory usage
2. **Prioritize tasks**: Ensure critical tasks get necessary resources
3. **Error recovery**: Implement graceful degradation when resources are limited
4. **Configuration tuning**: Adjust processing rates based on available resources

## Getting Help

If issues persist:

1. Check the troubleshooting guides for individual modules
2. Verify all prerequisites are met
3. Test components in isolation before integration
4. Review system logs for detailed error information
5. Consult the ROS 2, Isaac ROS, and LangChain documentation