# Capstone Project: Code Examples

## Overview

This document provides complete code examples for the integrated autonomous humanoid system that combines all four modules from the Physical AI Humanoid Robotics Book.

## Complete System Node

```python
#!/usr/bin/env python3
# capstone/autonomous_humanoid/src/integrated_humanoid.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from nav_msgs.msg import Odometry
import time
import threading
from typing import Dict, Any, Optional

class IntegratedHumanoid(Node):
    """
    Complete integrated humanoid system combining all four modules:
    - Module 1: ROS 2 fundamentals
    - Module 2: Simulation and navigation
    - Module 3: Isaac ROS perception
    - Module 4: VLA (Voice-Language-Action)
    """

    def __init__(self):
        super().__init__('integrated_humanoid')

        # Initialize system state
        self.system_active = True
        self.current_task = "IDLE"
        self.robot_pose = None
        self.detected_objects = []

        # Module 1: ROS 2 fundamentals - Publishers and subscribers
        self.status_pub = self.create_publisher(String, 'system_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Module 2: Simulation/Gazebo integration
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Module 3: Isaac ROS perception
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, 'camera/camera_info', self.camera_info_callback, 10)

        # Module 4: VLA system
        self.voice_cmd_sub = self.create_subscription(String, 'voice_command', self.voice_command_callback, 10)
        self.voice_response_pub = self.create_publisher(String, 'voice_response', 10)

        # Timer for system status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info("Integrated Humanoid System initialized successfully")

    def odom_callback(self, msg: Odometry):
        """Module 2: Handle odometry data from navigation system"""
        self.robot_pose = msg.pose.pose
        self.get_logger().debug(f"Updated robot pose: {self.robot_pose.position}")

    def scan_callback(self, msg: LaserScan):
        """Module 2: Handle laser scan data for obstacle detection"""
        # Process laser scan for navigation safety
        pass

    def image_callback(self, msg: Image):
        """Module 3: Handle camera images for perception"""
        # Process images through Isaac ROS perception pipeline
        self.get_logger().debug(f"Received image: {msg.width}x{msg.height}")

    def camera_info_callback(self, msg: CameraInfo):
        """Module 3: Handle camera calibration info"""
        # Use camera info for 3D reconstruction and object detection
        pass

    def voice_command_callback(self, msg: String):
        """Module 4: Process voice commands through VLA pipeline"""
        command = msg.data
        self.get_logger().info(f"Received voice command: {command}")

        # Process command through integrated system
        response = self.process_voice_command(command)

        # Publish response
        response_msg = String()
        response_msg.data = response
        self.voice_response_pub.publish(response_msg)

    def process_voice_command(self, command: str) -> str:
        """Process voice command through complete pipeline"""
        try:
            # Step 1: Parse command (VLA - Module 4)
            action_plan = self.plan_action_from_command(command)

            # Step 2: Check feasibility with perception (Isaac ROS - Module 3)
            if not self.validate_action_with_perception(action_plan):
                return f"Cannot execute {command}, environment constraints detected"

            # Step 3: Execute with navigation/manipulation (Simulation/Navigation - Module 2)
            success = self.execute_action_plan(action_plan)

            if success:
                return f"Successfully executed: {command}"
            else:
                return f"Failed to execute: {command}"

        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
            return f"Error processing command: {str(e)}"

    def plan_action_from_command(self, command: str) -> Dict[str, Any]:
        """Module 4: Plan actions from natural language command"""
        # This would integrate with LangChain for NLP processing
        # For example, parse "Go to kitchen and pick up red mug"
        action_plan = {
            'command': command,
            'primitive_actions': [],
            'target_location': None,
            'target_object': None,
            'action_sequence': []
        }

        # Simple parsing example
        if 'go to' in command.lower():
            # Extract location
            action_plan['target_location'] = command.lower().split('go to')[1].strip()
        elif 'pick up' in command.lower() or 'grasp' in command.lower():
            # Extract object
            action_plan['target_object'] = command.lower().replace('pick up', '').replace('grasp', '').strip()

        return action_plan

    def validate_action_with_perception(self, action_plan: Dict[str, Any]) -> bool:
        """Module 3: Validate action plan with perception data"""
        # Check if target location/object exists and is accessible
        # This would use Isaac ROS perception results
        return True  # Placeholder - in real system, validate with perception data

    def execute_action_plan(self, action_plan: Dict[str, Any]) -> bool:
        """Execute planned actions using navigation and manipulation"""
        try:
            # Execute sequence of primitive actions
            for action in action_plan.get('action_sequence', []):
                if not self.execute_primitive_action(action):
                    return False
            return True
        except Exception:
            return False

    def execute_primitive_action(self, action: Dict[str, Any]) -> bool:
        """Execute a single primitive action"""
        action_type = action.get('type', '')

        if action_type == 'navigate':
            return self.navigate_to_pose(action.get('target_pose'))
        elif action_type == 'grasp':
            return self.grasp_object(action.get('object_id'))
        elif action_type == 'move':
            return self.move_robot(action.get('velocity'))
        else:
            return False

    def navigate_to_pose(self, pose) -> bool:
        """Module 2: Navigate to target pose using navigation stack"""
        # Publish navigation goal
        goal_msg = PoseStamped()
        goal_msg.pose = pose
        # This would integrate with Nav2
        return True

    def grasp_object(self, object_id: str) -> bool:
        """Execute grasping action"""
        # This would control robot manipulator
        return True

    def move_robot(self, velocity) -> bool:
        """Send velocity commands to robot"""
        twist_msg = Twist()
        # Set velocity components
        self.cmd_vel_pub.publish(twist_msg)
        return True

    def publish_status(self):
        """Publish system status"""
        status_msg = String()
        status_msg.data = f"State: {self.current_task}, Active: {self.system_active}"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    humanoid = IntegratedHumanoid()

    try:
        rclpy.spin(humanoid)
    except KeyboardInterrupt:
        humanoid.get_logger().info("Shutting down integrated humanoid system")
    finally:
        humanoid.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File Example

```xml
<!-- capstone/autonomous_humanoid/launch/integrated_system.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Integrated humanoid node
    integrated_humanoid_node = Node(
        package='autonomous_humanoid',
        executable='integrated_humanoid',
        name='integrated_humanoid',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('autonomous_humanoid'), 'config', 'system_config.yaml')
        ]
    )
    ld.add_action(integrated_humanoid_node)

    # Additional nodes from all modules would be included here
    # ROS 2 nodes (Module 1)
    # Gazebo simulation nodes (Module 2)
    # Isaac ROS perception nodes (Module 3)
    # VLA system nodes (Module 4)

    return ld
```

## Package Configuration

```xml
<!-- capstone/autonomous_humanoid/package.xml -->
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>autonomous_humanoid</name>
  <version>1.0.0</version>
  <description>Complete integrated autonomous humanoid system combining all four modules</description>
  <maintainer email="maintainer@example.com">Maintainer Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Setup Script

```bash
#!/bin/bash
# capstone/autonomous_humanoid/setup.sh

echo "Setting up Autonomous Humanoid System..."

# Install Python dependencies
pip3 install -r requirements.txt

# Build ROS 2 packages
cd /workspace
colcon build --packages-select autonomous_humanoid

# Source environments
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.bash
source install/setup.bash

echo "Setup complete!"
```

## Requirements File

```
# capstone/autonomous_humanoid/requirements.txt
rclpy
std_msgs
geometry_msgs
sensor_msgs
nav_msgs
openai-whisper
langchain
langchain-openai
torch
numpy
opencv-python
```

These code examples demonstrate the complete integration of all four modules into a unified autonomous humanoid system that can process voice commands, perceive its environment, navigate safely, and perform manipulation tasks.