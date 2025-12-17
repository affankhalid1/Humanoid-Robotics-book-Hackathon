# ROS 2 Interfaces Contract: Physical AI Humanoid Robotics Book

## Overview
This document defines the ROS 2 interfaces (topics, services, actions) used throughout the book modules. All code examples must adhere to these interface specifications to ensure consistency and compatibility across modules.

## Message Types

### Custom Message Definitions

#### HumanoidState.msg
```text
# Current state of the humanoid robot
Header header
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
sensor_msgs/JointState joint_states
std_msgs/Float32[] joint_temperatures
bool in_motion
bool low_battery
string[] active_controllers
```

#### VoiceCommand.msg
```text
# Voice command received from speech recognition
Header header
string text
float32 confidence
string intent
string[] entities
string source_language
string target_language
```

#### ObjectDetection.msg
```text
# Object detection result
Header header
string[] object_names
float32[] confidences
sensor_msgs/RegionOfInterest[] bounding_boxes
geometry_msgs/PointStamped[] world_positions
sensor_msgs/Image cropped_images
```

## Topics

### Publisher Topics

#### `/humanoid/state`
- **Type**: `physical_ai_msgs/HumanoidState`
- **Frequency**: 10 Hz
- **Description**: Current state of the humanoid robot
- **Used by**: State monitoring nodes, visualization tools
- **QoS**: Reliable, keep_last(1)

#### `/voice/command`
- **Type**: `physical_ai_msgs/VoiceCommand`
- **Frequency**: Event-driven (when voice command received)
- **Description**: Voice commands from speech recognition
- **Used by**: Command processing nodes
- **QoS**: Reliable, keep_last(10)

#### `/object_detection/result`
- **Type**: `physical_ai_msgs/ObjectDetection`
- **Frequency**: 5-10 Hz (depending on processing)
- **Description**: Object detection results from perception system
- **Used by**: Task planning nodes
- **QoS**: Best effort, keep_last(5)

### Subscriber Topics

#### `/cmd_vel`
- **Type**: `geometry_msgs/Twist`
- **Frequency**: 10-50 Hz
- **Description**: Velocity commands for navigation
- **Published by**: Navigation stack
- **QoS**: Reliable, keep_last(1)

#### `/joint_commands`
- **Type**: `sensor_msgs/JointState`
- **Frequency**: 50-100 Hz
- **Description**: Joint position/velocity commands
- **Published by**: Motion planning nodes
- **QoS**: Reliable, keep_last(1)

#### `/sensor_data/imu`
- **Type**: `sensor_msgs/Imu`
- **Frequency**: 100 Hz
- **Description**: IMU sensor data for balance control
- **Published by**: IMU driver
- **QoS**: Reliable, keep_last(10)

## Services

### Navigation Services

#### `/navigate_to_pose`
- **Type**: `nav2_msgs/NavigateToPose`
- **Description**: Navigate humanoid to specified pose
- **Request**: Target pose with frame_id
- **Response**: Execution status and result
- **Timeout**: 30 seconds
- **Usage**: High-level navigation commands

#### `/get_path`
- **Type**: `nav_msgs/GetPlan`
- **Description**: Get navigation plan without executing
- **Request**: Start and goal poses
- **Response**: Navigation plan as Path message
- **Usage**: Path planning and visualization

### Perception Services

#### `/detect_objects`
- **Type**: Custom service definition
- **Description**: Detect objects in current field of view
- **Request**: Image and detection parameters
- **Response**: Object detection results
- **Timeout**: 5 seconds
- **Usage**: On-demand object detection

#### `/recognize_speech`
- **Type**: Custom service definition
- **Description**: Perform speech recognition on audio input
- **Request**: Audio data or file path
- **Response**: Recognized text with confidence
- **Timeout**: 10 seconds
- **Usage**: Voice command processing

## Actions

### Manipulation Actions

#### `/move_to_pose`
- **Type**: `control_msgs/FollowJointTrajectory`
- **Description**: Move manipulator to specific pose
- **Goal**: Target joint positions/velocities
- **Feedback**: Current progress and trajectory state
- **Result**: Success/failure of execution
- **Timeout**: 30 seconds
- **Usage**: Arm and hand manipulation

#### `/grasp_object`
- **Type**: Custom action definition
- **Description**: Execute grasp on detected object
- **Goal**: Object ID or pose to grasp
- **Feedback**: Grasp approach progress
- **Result**: Success/failure of grasp
- **Timeout**: 15 seconds
- **Usage**: Object manipulation tasks

### Navigation Actions

#### `/navigate_with_feedback`
- **Type**: `nav2_msgs/NavigateWithReplanning`
- **Description**: Navigate with dynamic replanning
- **Goal**: Target pose with navigation parameters
- **Feedback**: Current pose, remaining distance, replanning status
- **Result**: Final navigation result
- **Timeout**: 60 seconds
- **Usage**: Complex navigation with obstacle avoidance

## Quality of Service (QoS) Profiles

### Default Profiles

#### State Topics (High Frequency)
- **Reliability**: Reliable
- **Durability**: Volatile
- **History**: Keep last 1
- **Depth**: 1

#### Sensor Topics (High Frequency)
- **Reliability**: Best effort
- **Durability**: Volatile
- **History**: Keep last 10
- **Depth**: 10

#### Command Topics (Medium Frequency)
- **Reliability**: Reliable
- **Durability**: Volatile
- **History**: Keep last 1
- **Depth**: 1

#### Event Topics (Low Frequency)
- **Reliability**: Reliable
- **Durability**: Volatile
- **History**: Keep last 10
- **Depth**: 10

## Frame Conventions

### Coordinate Frames
- **`base_link`**: Robot's base coordinate frame
- **`odom`**: Odometry reference frame
- **`map`**: Global map frame
- **`camera_rgb_optical_frame`**: RGB camera optical frame
- **`imu_link`**: IMU sensor frame
- **`left_hand`**, **`right_hand`**: End-effector frames

### Transform Conventions
- All transforms must be published to TF tree
- Static transforms in URDF when possible
- Dynamic transforms via `tf2_ros` broadcasters
- Consistent naming following ROS 2 standards

## Error Handling

### Standard Error Codes
- `0`: Success
- `1`: General failure
- `2`: Invalid parameters
- `3`: Timeout
- `4`: Hardware fault
- `5`: Collision detected
- `6`: Goal aborted
- `7`: Preemption requested

### Error Messages Format
- All error messages must be human-readable
- Include error code and descriptive text
- Log errors to ROS 2 logging system
- Provide recovery suggestions when possible

## Validation Requirements

### Interface Compliance
- All publishers/subscribers must use defined message types
- Service and action interfaces must match specifications
- QoS profiles must match documented requirements
- Topic names must follow ROS 2 naming conventions

### Performance Requirements
- Publishers must maintain specified frequencies
- Services must respond within timeout limits
- Actions must provide feedback at minimum rate of 1 Hz
- Memory usage must remain within documented limits

## Versioning
- Interface version: 1.0.0
- Compatible with ROS 2 Humble
- Changes to interfaces will follow semantic versioning
- Breaking changes require major version increment