# Module 2: Simulation Troubleshooting Guide

## Common Installation Issues

### Gazebo Installation Problems

**Issue**: Gazebo fails to launch or crashes immediately
**Solution**:
- Ensure you have OpenGL 3.3+ support: `glxinfo | grep "OpenGL version"`
- Install proper graphics drivers for your GPU
- Try running with software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`

**Issue**: ROS 2 packages not found for Gazebo integration
**Solution**:
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
source /opt/ros/humble/setup.bash
```

### Docker Setup Issues

**Issue**: Gazebo doesn't display properly in Docker
**Solution**:
- Ensure X11 forwarding is enabled
- Run Docker with proper display permissions: `xhost +local:docker`
- Use the provided docker-compose configuration

## Simulation Performance Issues

### Slow Simulation Speed

**Issue**: Simulation runs significantly slower than real-time
**Solutions**:
1. Increase physics step size (trade accuracy for speed):
   ```xml
   <max_step_size>0.01</max_step_size>
   ```
2. Reduce real-time update rate
3. Simplify collision geometries
4. Reduce number of active sensors

### Physics Instability

**Issue**: Robot falls through ground or exhibits unstable behavior
**Solutions**:
1. Check mass properties in URDF:
   ```xml
   <inertial>
     <mass value="1.0"/>
     <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
   </inertial>
   ```
2. Adjust surface parameters:
   ```xml
   <surface>
     <contact>
       <ode>
         <kp>1000000000000.0</kp>
         <kd>1.0</kd>
       </ode>
     </contact>
   </surface>
   ```

## URDF Model Issues

### Model Not Loading

**Issue**: Robot model fails to load in Gazebo
**Solutions**:
1. Validate URDF syntax: `check_urdf /path/to/robot.urdf`
2. Check for missing mesh files
3. Verify joint limits and types
4. Ensure proper parent-child relationships

### Visual/Collision Mismatch

**Issue**: Visual model doesn't match collision model
**Solution**: Ensure both visual and collision elements are properly defined:
```xml
<link name="link_name">
  <visual>
    <geometry>
      <mesh filename="mesh.stl"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <mesh filename="collision.stl"/>
    </geometry>
  </collision>
</link>
```

## Sensor Troubleshooting

### Camera Sensor Issues

**Issue**: Camera sensor not publishing images
**Solutions**:
1. Check namespace configuration in plugin
2. Verify update rate is set appropriately
3. Ensure proper camera link orientation

**Issue**: Distorted or incorrect images
**Solutions**:
1. Check camera intrinsics: horizontal_fov, image width/height
2. Verify near/far clipping distances
3. Ensure proper lighting in the world

### LiDAR Sensor Issues

**Issue**: LiDAR data is sparse or incorrect
**Solutions**:
1. Check scan parameters: samples, min_angle, max_angle
2. Verify range settings: min, max, resolution
3. Ensure proper update rate

### IMU Sensor Issues

**Issue**: IMU readings are noisy or incorrect
**Solutions**:
1. Check noise parameters match real sensor specifications
2. Verify coordinate frame alignment
3. Ensure proper update rate (typically 100Hz+)

## ROS 2 Integration Issues

### Topic Communication Problems

**Issue**: Topics not connecting between ROS 2 and Gazebo
**Solutions**:
1. Verify namespaces match between plugins and subscribers
2. Check Quality of Service (QoS) settings
3. Ensure both systems are on the same ROS domain

**Issue**: High latency in sensor data
**Solutions**:
1. Use appropriate QoS profiles (often best effort for sensors)
2. Reduce update rates if not needed
3. Check network configuration

## Gazebo-Specific Issues

### World Loading Problems

**Issue**: World file fails to load
**Solutions**:
1. Validate SDF syntax: `gz sdf -k world_file.sdf`
2. Check for missing model dependencies
3. Verify physics engine configuration

### Plugin Loading Failures

**Issue**: Custom plugins fail to load
**Solutions**:
1. Verify plugin library path and filename
2. Check plugin dependencies
3. Ensure proper plugin class registration
4. Look for error messages in Gazebo console

## Performance Optimization

### Reducing CPU Usage

1. Lower physics update rate
2. Reduce number of active sensors
3. Simplify collision meshes
4. Use simpler physics engine (ODE vs Bullet)

### Reducing Memory Usage

1. Reduce sensor image resolution
2. Limit number of models in simulation
3. Use instanced rendering for repeated objects

## Debugging Tips

### Enable Verbose Logging

```bash
gz sim -v 4 world_file.sdf  # Highest verbosity
```

### Check Model Information

```bash
gz model --info  # List all models in simulation
gz topic -l       # List all available topics
```

### Validate URDF

```bash
check_urdf /path/to/robot.urdf
urdf_to_graphiz /path/to/robot.urdf
```

## Common Error Messages

### "Model not found in model database"
- Ensure model files are in GAZEBO_MODEL_PATH
- Verify model URI is correct in SDF files

### "Plugin failed to load"
- Check library file exists and is executable
- Verify plugin filename matches expected format
- Check for missing dependencies

### "Joint state publisher not working"
- Ensure joint names match between URDF and controller
- Verify joint state topic is being published
- Check controller configuration

## Quick Fixes

### Reset Simulation
- Use Gazebo GUI controls to reset model positions
- Or restart the simulation completely

### Reconnect ROS 2
- Source ROS 2 environment: `source /opt/ros/humble/setup.bash`
- Check ROS domain: `echo $ROS_DOMAIN_ID`
- Restart ROS 2 nodes if needed

## Getting Help

If issues persist:

1. Check Gazebo documentation: http://gazebosim.org/tutorials
2. ROS 2 Gazebo integration: https://github.com/ros-simulation/gazebo_ros_pkgs
3. Community forums and Q&A sites
4. Verify your configuration against working examples in the repository