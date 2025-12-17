---
sidebar_position: 3
---

# Chapter 2: Robot Modeling

## Introduction

Robot modeling is the process of creating digital representations of physical robots for simulation. The Unified Robot Description Format (URDF) is the standard format used in ROS for representing robot models. A well-designed robot model is crucial for accurate simulation and successful transfer to real hardware.

## URDF Fundamentals

URDF (Unified Robot Description Format) is an XML format that describes robots. It includes:

- **Links**: Rigid parts of the robot (e.g., chassis, arms, wheels)
- **Joints**: Connections between links (e.g., revolute, prismatic, fixed)
- **Visual**: How the robot appears in simulation
- **Collision**: How the robot interacts with the environment
- **Inertial**: Physical properties for physics simulation

### Basic URDF Structure

```xml
<robot name="my_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Connected link -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Link Properties

### Visual Properties
- **Geometry**: Shape definition (box, cylinder, sphere, mesh)
- **Material**: Color and texture properties
- **Origin**: Position and orientation relative to joint

### Collision Properties
- **Geometry**: Defines collision boundaries
- **Origin**: Position and orientation relative to joint

### Inertial Properties
- **Mass**: Physical mass of the link
- **Inertia**: Inertia tensor values (ixx, ixy, ixz, iyy, iyz, izz)

## Joint Types

### Fixed Joint
- No degrees of freedom
- Used for attaching static components

### Revolute Joint
- Single rotational degree of freedom
- Limited by upper/lower position limits
- Used for wheels, arm joints

### Continuous Joint
- Unlimited rotational motion
- Similar to revolute but without limits
- Used for wheels that can rotate indefinitely

### Prismatic Joint
- Single linear degree of freedom
- Limited by position limits
- Used for linear actuators

### Floating Joint
- Six degrees of freedom
- Used for floating objects

## Creating a Humanoid Robot Model

Let's build a simplified humanoid robot model:

### Head Section
```xml
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
  </inertial>
</link>
```

### Torso Section
```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.4"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.4"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
  </inertial>
</link>
```

## Gazebo-Specific Extensions

URDF can be extended with Gazebo-specific tags:

### Material Definition
```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
</gazebo>
```

### Plugin Integration
```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <namespace>/robot</namespace>
    </ros>
    <update_rate>30</update_rate>
  </plugin>
</gazebo>
```

## Xacro for Complex Models

Xacro (XML Macros) helps manage complex models by allowing:

- Variable definitions
- Macros for repeated structures
- Mathematical expressions
- File inclusion

Example Xacro usage:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="wheel" params="suffix parent x y color">
    <link name="${suffix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
        <material name="${color}"/>
      </visual>
    </link>

    <joint name="${parent}_to_${suffix}_wheel" type="continuous">
      <parent link="${parent}"/>
      <child link="${suffix}_wheel"/>
      <origin xyz="${x} ${y} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <xacro:wheel suffix="left" parent="base_link" x="0" y="0.15" color="blue"/>
  <xacro:wheel suffix="right" parent="base_link" x="0" y="-0.15" color="blue"/>
</robot>
```

## Best Practices for Robot Modeling

1. **Mass Properties**: Accurately model mass and inertia for realistic physics
2. **Collision vs Visual**: Use simpler geometries for collision than visual
3. **Joint Limits**: Set appropriate limits based on real hardware
4. **Mesh Optimization**: Simplify meshes for better performance
5. **Consistent Units**: Use consistent units throughout the model
6. **Validation**: Use tools like `check_urdf` to validate models

## Model Validation

Validate your URDF model using:
```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Parse and display model info
urdf_to_graphiz /path/to/robot.urdf
```

## Troubleshooting Common Issues

- **Model Instability**: Check mass properties and joint limits
- **Mesh Loading**: Ensure mesh paths are correct and files exist
- **Joint Direction**: Verify joint axes are correctly oriented
- **Collision Issues**: Ensure collision geometries are properly defined
- **Performance**: Simplify collision geometries if simulation is slow

## Advanced Topics

### Transmission Elements
Define how actuators connect to joints:
```xml
<transmission name="wheel_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Sensors in URDF
Integrate sensors directly in the robot model:
```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
      <ros>
        <namespace>/camera</namespace>
        <remapping>~/image_raw:=image</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Summary

In this chapter, we've covered:
- URDF fundamentals and structure
- Link and joint properties
- Gazebo-specific extensions
- Xacro for complex models
- Best practices and validation techniques
- Advanced topics like transmissions and sensors

In the next chapter, we'll explore physics properties and sensor integration in detail.