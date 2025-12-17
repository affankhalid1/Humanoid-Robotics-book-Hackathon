# Module 3: Isaac ROS Troubleshooting Guide

## Installation Issues

### Isaac ROS Installation Problems

**Issue**: Isaac ROS packages not found after installation
**Solution**:
```bash
# Verify ROS 2 Humble is properly sourced
source /opt/ros/humble/setup.bash

# Check if Isaac ROS repository is added
apt list --installed | grep isaac

# Reinstall if needed
sudo apt install ros-humble-isaac-ros-dev
```

**Issue**: CUDA not working with Isaac ROS
**Solution**:
- Verify CUDA installation: `nvcc --version`
- Check GPU availability: `nvidia-smi`
- Ensure CUDA version compatibility with Isaac ROS

### Isaac Sim Installation Issues

**Issue**: Isaac Sim fails to launch
**Solution**:
- Check GPU driver compatibility
- Verify sufficient VRAM (8GB+ recommended)
- Ensure proper display permissions

## GPU and Performance Issues

### GPU Memory Problems

**Issue**: Out of memory errors during perception processing
**Solutions**:
1. Reduce input image resolution
2. Decrease batch size in deep learning models
3. Optimize feature count in SLAM algorithms
4. Monitor GPU memory usage: `nvidia-smi`

**Issue**: Low frame rate for real-time processing
**Solutions**:
1. Optimize kernel launch parameters
2. Reduce algorithm complexity
3. Use lower precision (FP16) when possible
4. Profile with Nsight Systems to identify bottlenecks

### CUDA Runtime Errors

**Issue**: CUDA kernel launch failures
**Solutions**:
1. Check GPU compute capability requirements
2. Verify CUDA driver version compatibility
3. Ensure proper GPU memory allocation
4. Use unified memory for easier management

## Perception Pipeline Issues

### Object Detection Problems

**Issue**: Poor detection accuracy
**Solutions**:
1. Verify camera calibration
2. Check lighting conditions
3. Adjust confidence thresholds
4. Retrain models with domain-specific data

**Issue**: High latency in detection pipeline
**Solutions**:
1. Use TensorRT optimized models
2. Optimize batch sizes
3. Implement asynchronous processing
4. Reduce image resolution if possible

### Feature Tracking Issues

**Issue**: Feature tracking failure in cuvSLAM
**Solutions**:
1. Ensure sufficient texture in environment
2. Adjust feature detection parameters
3. Verify camera calibration
4. Check for motion blur in images

## SLAM and Navigation Issues

### cuvSLAM Problems

**Issue**: SLAM drift and inaccurate mapping
**Solutions**:
1. Optimize keyframe selection
2. Improve loop closure detection
3. Verify sensor calibration
4. Adjust optimization parameters

**Issue**: Tracking failure in cuvSLAM
**Solutions**:
1. Increase feature count
2. Adjust tracking thresholds
3. Verify camera exposure settings
4. Check for fast motion causing blur

### Nav2 Integration Issues

**Issue**: Navigation fails with cuvSLAM maps
**Solutions**:
1. Verify TF frame relationships
2. Check map resolution and inflation
3. Ensure proper coordinate frame alignment
4. Validate sensor data quality

## Docker and Container Issues

### Isaac Sim Container Problems

**Issue**: Isaac Sim doesn't render properly in Docker
**Solutions**:
1. Enable GPU access: `--gpus all`
2. Set proper display permissions: `xhost +local:docker`
3. Check X11 forwarding: `-v /tmp/.x11-unix:/tmp/.x11-unix`
4. Verify OpenGL support in container

**Issue**: Container fails to start with GPU access
**Solutions**:
1. Install nvidia-docker2: `sudo apt install nvidia-docker2`
2. Restart Docker: `sudo systemctl restart docker`
3. Verify NVIDIA runtime: `docker info | grep -i nvidia`

## Sensor Integration Issues

### Camera Calibration Problems

**Issue**: Incorrect depth estimation
**Solutions**:
1. Recalibrate camera using standard patterns
2. Verify distortion parameters
3. Check camera pose in URDF
4. Validate calibration on test images

**Issue**: Synchronization between sensors
**Solutions**:
1. Use hardware triggering when possible
2. Implement software synchronization
3. Check timestamp accuracy
4. Verify sensor frame rates match

## Quality of Service Issues

### Message Timing Problems

**Issue**: Perception data drops or delays
**Solutions**:
1. Adjust QoS profiles for best effort
2. Increase buffer sizes
3. Optimize network bandwidth
4. Check for CPU/GPU bottlenecks

**Issue**: SLAM-Nav2 communication issues
**Solutions**:
1. Verify TF tree integrity
2. Check message rates and timing
3. Ensure proper frame transformations
4. Validate coordinate system alignment

## Debugging Tools

### Isaac ROS Logging

Enable detailed logging:
```bash
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
export RCUTILS_LOGGING_BUFFERED_STREAM=1
```

### Performance Profiling

Use NVIDIA tools for optimization:
```bash
# Profile GPU usage
nvidia-ml-py3

# Profile CUDA kernels
nsys profile --trace=cuda,nvtx ros2 run ...

# Profile CPU-GPU synchronization
ncu --target-processes all ros2 run ...
```

## Common Error Messages

### "Failed to create CUDA context"
- Check GPU driver installation
- Verify CUDA runtime compatibility
- Ensure GPU is not being used by another process

### "Isaac ROS node failed to initialize"
- Check required dependencies
- Verify GPU memory availability
- Check Isaac ROS package installation

### "SLAM tracking lost"
- Ensure sufficient visual features
- Check camera exposure settings
- Verify sensor calibration
- Reduce robot motion speed

## Performance Optimization

### GPU Memory Management

- Monitor memory usage: `nvidia-smi -l 1`
- Implement memory pooling
- Use unified memory for easier management
- Optimize data structures for GPU access

### Pipeline Optimization

- Use asynchronous processing
- Implement pipeline parallelism
- Optimize kernel launch parameters
- Minimize CPU-GPU synchronization

## Getting Help

If issues persist:

1. Check Isaac ROS documentation: https://nvidia-isaac-ros.github.io/
2. Isaac Sim documentation: https://docs.omniverse.nvidia.com/isaacsim/
3. ROS 2 integration guides
4. NVIDIA developer forums
5. Community support channels