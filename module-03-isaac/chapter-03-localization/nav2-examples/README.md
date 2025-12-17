# Isaac ROS Nav2 Integration Example

This example demonstrates the integration of cuvSLAM with Nav2 for complete navigation.

## Overview

This example shows how to:
- Integrate cuvSLAM with Nav2
- Configure navigation parameters
- Plan and execute paths in real-time
- Handle dynamic obstacles

## Prerequisites

- Isaac ROS with cuvSLAM
- Nav2 installed
- NVIDIA GPU for cuvSLAM
- Robot with proper URDF

## Launch the Example

```bash
# Source ROS 2 and Isaac ROS
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.bash

# Launch cuvSLAM and Nav2 integration
ros2 launch nav2_cuvslam_integration nav2_cuvslam.launch.py
```

## Configuration

The example uses the following Nav2 configuration:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_mppi_controller::Controller"
      time_steps: 50
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.05
      wz_std: 0.3
      vx_max: 0.5
      vx_min: -0.5
      vy_max: 0.3
      wz_max: 1.0
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      simulation_timeout: 15.0
      speed_scaling_factor: 0.25
      debug_visualizations: false
      publish_cost_grid_pc: false
      transform_tolerance: 0.5
      alpha: 0.0
      beta: 0.0
      gamma: 20.0
      motion_model: "DiffDrive"
      visualize_ll: false
      visualize_controls_l: 10
      controls_sampling_period: 0.2
      nx: 3
      nu: 2
      horizon_window: 1.0
      dt: 0.2
      lambda: 1.0
      mu: 1.0
      nu_1: 1.0
      nu_2: 1.0
      nu_3: 1.0
      obstacle_cost: 1.0
      goal_cost: 1.0
      reference_cost: 1.0