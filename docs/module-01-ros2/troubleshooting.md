---
sidebar_position: 6
---

# Module 1: Troubleshooting Guide

## Common Issues and Solutions

This guide provides solutions to common problems encountered when working with ROS 2 in Module 1.

### Installation Issues

**Problem**: Cannot install ROS 2 Humble on Ubuntu 22.04
**Solution**:
1. Verify your Ubuntu version: `lsb_release -a`
2. Follow the official installation guide: https://docs.ros.org/en/humble/Installation.html
3. Make sure to set up your locale: `locale-gen en_US.UTF-8`
4. If getting GPG errors, update your keys:
   ```bash
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

**Problem**: Python packages not found after ROS 2 installation
**Solution**:
1. Source the ROS 2 environment: `source /opt/ros/humble/setup.bash`
2. Add to your ~/.bashrc file to make it permanent:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Node Communication Issues

**Problem**: Nodes cannot communicate across different terminals
**Solution**:
1. Make sure you're using the same ROS_DOMAIN_ID in all terminals
2. Set it explicitly: `export ROS_DOMAIN_ID=0`
3. Or use different domain IDs to separate systems: `export ROS_DOMAIN_ID=42`

**Problem**: Publisher/Subscriber not connecting
**Solution**:
1. Check if nodes are running: `ros2 node list`
2. Verify topic names: `ros2 topic list`
3. Check topic types: `ros2 topic info /topic_name`
4. Ensure both nodes are on the same domain ID
5. Verify network configuration if running across machines

### DDS/RMW Issues

**Problem**: Multiple RMW implementations causing conflicts
**Solution**:
1. Check available RMW implementations: `ros2 doctor --report`
2. Set a specific RMW: `export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp`
3. This is often the most stable choice for development

**Problem**: Nodes not discovering each other
**Solution**:
1. Check firewall settings (usually UDP ports 7400+)
2. Verify network interface: `ros2 topic list --spin-time 1`
3. Try setting specific network interface: `export ROS_LOCALHOST_ONLY=1` (for single machine)

### Python rclpy Issues

**Problem**: "No module named rclpy" error
**Solution**:
1. Make sure ROS 2 environment is sourced: `source /opt/ros/humble/setup.bash`
2. Install Python development tools: `sudo apt install python3-dev`
3. Verify Python version compatibility (3.10+ for Humble)

**Problem**: Node hangs or doesn't respond
**Solution**:
1. Check if you're calling `rclpy.spin()` in your main function
2. Ensure proper exception handling in callbacks
3. Verify that your node isn't blocking in a callback

### Build and Dependency Issues

**Problem**: colcon build fails with dependency errors
**Solution**:
1. Install rosdep and dependencies:
   ```bash
   sudo apt install python3-rosdep
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```
2. Make sure you're in the workspace root directory
3. Clean build if needed: `rm -rf build/ install/ log/`

**Problem**: Custom message types not found
**Solution**:
1. Make sure your package.xml includes the message dependencies
2. Add to CMakeLists.txt: `find_package(rosidl_default_generators REQUIRED)`
3. Run `colcon build` again after message definition changes

### Performance Issues

**Problem**: High CPU usage with ROS 2 nodes
**Solution**:
1. Check timer frequencies in your nodes
2. Use appropriate QoS settings for your use case
3. Monitor with: `ros2 doctor --report`

**Problem**: Memory leaks in long-running nodes
**Solution**:
1. Properly destroy nodes in your shutdown sequence
2. Use context managers where appropriate
3. Monitor with: `ros2 lifecycle list` for lifecycle nodes

### Docker-Related Issues

**Problem**: GUI applications not working in Docker
**Solution**:
1. Ensure X11 forwarding is set up: `xhost +local:docker`
2. Run Docker with proper display settings
3. For Windows/WSL2, set up VcXsrv or similar X server

**Problem**: Permission denied when accessing host files from container
**Solution**:
1. Check user ID mapping between host and container
2. Use appropriate volume mounting options
3. Consider using the same user ID in container as host

### Testing and Debugging

**Problem**: Cannot debug ROS 2 nodes effectively
**Solution**:
1. Use `rclpy.logging` for proper ROS 2 logging
2. Use `ros2 doctor` for system diagnostics
3. Use `ros2 bag` to record and replay data for debugging

**Problem**: Services or actions not responding
**Solution**:
1. Verify service/action names: `ros2 service list` or `ros2 action list`
2. Check types: `ros2 service type <service_name>`
3. Test with command line tools: `ros2 service call <name> <type> <request>`

### Development Environment

**Problem**: IDE not recognizing ROS 2 packages
**Solution**:
1. Source ROS 2 in your IDE's terminal environment
2. Install ROS 2 extensions for your IDE
3. Set Python path to include ROS 2 packages

**Problem**: Code completion not working with ROS 2 messages
**Solution**:
1. Make sure your workspace is built: `colcon build`
2. Source the install setup: `source install/setup.bash`
3. Configure your IDE to use the workspace Python environment

## Getting Help

If you encounter issues not covered here:

1. Check the ROS 2 documentation: https://docs.ros.org/
2. Search ROS Answers: https://answers.ros.org/
3. Visit ROS Discourse: https://discourse.ros.org/
4. Check the specific package documentation
5. Use `ros2 doctor` for system diagnostics

## Verification Steps

To verify your ROS 2 installation is working properly:

1. Source the environment: `source /opt/ros/humble/setup.bash`
2. Run a simple test: `ros2 topic list`
3. Check the installation: `ros2 doctor --report`
4. Try the basic tutorial: `ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_cpp listener` in separate terminals

## Best Practices for Avoiding Issues

1. Always source the ROS 2 environment before working
2. Use consistent naming conventions
3. Implement proper error handling
4. Test components independently before integration
5. Use version control for your code
6. Document your custom message and service definitions
7. Keep your system and ROS 2 installation up to date