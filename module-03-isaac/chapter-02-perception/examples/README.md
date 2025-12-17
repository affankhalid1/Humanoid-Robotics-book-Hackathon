# Isaac ROS Perception Example

This example demonstrates GPU-accelerated object detection using Isaac ROS.

## Overview

This example shows how to:
- Set up an Isaac ROS perception pipeline
- Configure GPU-accelerated object detection
- Process camera images in real-time
- Visualize detection results

## Prerequisites

- Isaac ROS installed
- NVIDIA GPU with CUDA support
- Camera source (real or simulated)

## Launch the Example

```bash
# Source ROS 2 and Isaac ROS
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.bash

# Launch the perception pipeline
ros2 launch perception_example perception_pipeline.launch.py
```

## Configuration

The example uses DetectNet for object detection with the following configuration:

```yaml
detectnet:
  model_name: "ssd_mobilenet_v2_coco"
  input_tensor: "input"
  input_format: "bgr8"
  confidence_threshold: 0.5
  enable_profiling: false
```

## Performance

This example achieves 60+ FPS on RTX 3080 GPU with proper configuration.