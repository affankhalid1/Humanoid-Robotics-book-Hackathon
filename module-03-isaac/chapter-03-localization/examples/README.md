# cuvSLAM Example

This example demonstrates GPU-accelerated Visual SLAM using cuvSLAM.

## Overview

This example shows how to:
- Set up a cuvSLAM pipeline
- Configure GPU-accelerated visual SLAM
- Process camera images for real-time mapping
- Integrate with Nav2 for navigation

## Prerequisites

- Isaac ROS with cuvSLAM installed
- NVIDIA GPU with CUDA support
- Calibrated camera source
- Nav2 installed for navigation

## Launch the Example

```bash
# Source ROS 2 and Isaac ROS
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.bash

# Launch cuvSLAM with navigation
ros2 launch cuvslam_example cuvslam_navigation.launch.py
```

## Configuration

The example uses the following cuvSLAM configuration:

```yaml
cuvslam:
  max_features: 2000
  min_feature_distance: 10
  feature_quality_threshold: 0.01
  max_tracking_error: 10.0
  min_inliers: 15
  reprojection_threshold: 3.0
  max_iterations: 100
  convergence_threshold: 1e-6
  cuda_device: 0
  memory_pool_size: 512
  max_frame_rate: 60
```

## Performance

This example achieves 60+ FPS on RTX 3080 GPU with proper configuration.