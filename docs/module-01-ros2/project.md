---
sidebar_position: 7
---

# Module 1 Project: Hello Robot World

## Project Overview

In this project, you'll implement the "Hello Robot World" system that integrates all the concepts learned in Module 1. You'll create a simple robot node that publishes its status and responds to commands, demonstrating ROS 2 fundamentals including nodes, topics, services, and actions.

## Learning Objectives

By completing this project, you will:

- Create multiple interconnected ROS 2 nodes
- Implement publisher-subscriber communication patterns
- Use services for request-response interactions
- Apply Quality of Service (QoS) settings appropriately
- Structure a basic robotic application using ROS 2 patterns
- Test and debug a multi-node ROS 2 system

## Project Requirements

### Core Components

1. **Robot State Publisher Node**
   - Publishes the robot's current status (position, battery level, operational state)
   - Publishes to `/robot_status` topic
   - Uses appropriate QoS settings for status updates

2. **Command Subscriber Node**
   - Subscribes to `/robot_commands` topic
   - Processes commands like "move_forward", "turn_left", "stop"
   - Updates robot state based on commands

3. **Battery Monitoring Service**
   - Provides a service to query current battery level
   - Service name: `/get_battery_level`
   - Service type: `std_srvs/srv/Trigger` (with battery level in response message)

4. **Navigation Action Server**
   - Implements a navigation action that moves the robot to a goal position
   - Action name: `/navigate_to_pose`
   - Provides feedback on navigation progress
   - Supports cancellation of navigation goals

5. **Robot Controller Node**
   - Coordinates between all components
   - Manages robot state transitions
   - Handles safety checks

### Technical Specifications

- Use Python with rclpy client library
- Follow ROS 2 naming conventions
- Implement proper error handling and logging
- Use appropriate QoS settings for different types of data
- Structure code in a modular, maintainable way

## Implementation Steps

### Step 1: Project Setup

1. Create a new ROS 2 package for your project:
   ```bash
   mkdir -p ~/ros2_workspace/src/hello_robot_world
   cd ~/ros2_workspace/src/hello_robot_world
   ```

2. Create the basic directory structure:
   ```
   hello_robot_world/
   ├── CMakeLists.txt
   ├── package.xml
   ├── setup.py
   └── hello_robot_world/
       ├── __init__.py
       ├── robot_state_publisher.py
       ├── command_subscriber.py
       ├── battery_service.py
       ├── navigation_action.py
       └── robot_controller.py
   ```

### Step 2: Robot State Publisher

Create a node that publishes robot status:

```python
# hello_robot_world/robot_state_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Publisher for robot status
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)

        # Timer to publish status periodically
        self.timer = self.create_timer(1.0, self.publish_status)

        # Robot state variables
        self.position = {'x': 0.0, 'y': 0.0}
        self.battery_level = 100.0
        self.operational = True

    def publish_status(self):
        msg = String()

        # Simulate battery drain
        self.battery_level = max(0.0, self.battery_level - 0.1)

        # Create status message
        status_msg = f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f}), "
        status_msg += f"Battery: {self.battery_level:.1f}%, "
        status_msg += f"Operational: {self.operational}"

        msg.data = status_msg
        self.status_publisher.publish(msg)

        self.get_logger().info(f'Published status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Command Subscriber

Create a node that subscribes to commands:

```python
# hello_robot_world/command_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')

        # Subscriber for robot commands
        self.command_subscriber = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )

        # Publisher for movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('Command subscriber node initialized')

    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')

        # Create twist message based on command
        twist_msg = Twist()

        if command == 'move_forward':
            twist_msg.linear.x = 0.5
            twist_msg.angular.z = 0.0
        elif command == 'turn_left':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5
        elif command == 'stop':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(f'Published movement command: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = CommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Battery Service

Create a service to query battery level:

```python
# hello_robot_world/battery_service.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class BatteryService(Node):
    def __init__(self):
        super().__init__('battery_service')

        # Service server for battery queries
        self.srv = self.create_service(
            Trigger,
            'get_battery_level',
            self.battery_callback
        )

        # Simulated battery level
        self.battery_level = 85.0

        self.get_logger().info('Battery service initialized')

    def battery_callback(self, request, response):
        # Simulate battery level check
        response.success = True
        response.message = f'Battery level: {self.battery_level:.1f}%'

        self.get_logger().info(f'Battery service called: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BatteryService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5: Navigation Action

Create a navigation action server:

```python
# hello_robot_world/navigation_action.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
# Note: Using a simple action structure since NavigateToPose might not be available
# In practice, you'd use nav2_msgs/action/NavigateToPose

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # For this example, we'll simulate the action using a service
        # since creating custom actions requires more setup
        from std_srvs.srv import Trigger
        self.nav_srv = self.create_service(
            Trigger,
            'navigate_to_goal',
            self.navigate_callback
        )

        self.get_logger().info('Navigation service initialized (simulated action)')

    def navigate_callback(self, request, response):
        # Simulate navigation to goal
        # In a real action, you would provide feedback during execution
        self.get_logger().info('Navigating to goal...')

        # Simulate navigation time
        import time
        time.sleep(2)  # Simulate 2 seconds navigation time

        response.success = True
        response.message = 'Navigation completed successfully'

        self.get_logger().info('Navigation completed')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 6: Robot Controller

Create the main controller node:

```python
# hello_robot_world/robot_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers and subscribers
        self.command_publisher = self.create_publisher(String, 'robot_commands', 10)
        self.status_subscriber = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10
        )

        # Service client for battery
        self.battery_client = self.create_client(Trigger, 'get_battery_level')

        # Timer for periodic tasks
        self.control_timer = self.create_timer(5.0, self.periodic_control)

        self.get_logger().info('Robot controller initialized')

    def status_callback(self, msg):
        self.get_logger().info(f'Received status update: {msg.data}')

    def periodic_control(self):
        # Periodically check battery and request status
        if self.battery_client.service_is_ready():
            future = self.battery_client.call_async(Trigger.Request())
            future.add_done_callback(self.battery_response_callback)

    def battery_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Battery check result: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 7: Package Configuration

Create the `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>hello_robot_world</name>
  <version>0.0.1</version>
  <description>Hello Robot World project for Module 1</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_srvs</depend>

  <exec_depend>python3</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Create the `setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'hello_robot_world'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Hello Robot World project for Module 1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_publisher = hello_robot_world.robot_state_publisher:main',
            'command_subscriber = hello_robot_world.command_subscriber:main',
            'battery_service = hello_robot_world.battery_service:main',
            'navigation_action = hello_robot_world.navigation_action:main',
            'robot_controller = hello_robot_world.robot_controller:main',
        ],
    },
)
```

## Testing the Project

### Step 1: Build the Package
```bash
cd ~/ros2_workspace
colcon build --packages-select hello_robot_world
source install/setup.bash
```

### Step 2: Run Individual Nodes
Open separate terminals and run each node:
```bash
# Terminal 1
ros2 run hello_robot_world robot_state_publisher

# Terminal 2
ros2 run hello_robot_world command_subscriber

# Terminal 3
ros2 run hello_robot_world battery_service

# Terminal 4
ros2 run hello_robot_world robot_controller
```

### Step 3: Test Communication
```bash
# Send a command
echo "move_forward" | ros2 topic pub /robot_commands std_msgs/String "data: 'move_forward'"

# Check status
ros2 topic echo /robot_status

# Query battery
ros2 service call /get_battery_level std_srvs/srv/Trigger
```

## Project Deliverables

1. **Complete Source Code**: All Python files for the robot system
2. **Package Configuration**: Properly configured package.xml and setup.py
3. **Documentation**: Comments and docstrings explaining the code
4. **Test Results**: Evidence that all components work together
5. **Reflection Report**: A short report on challenges faced and lessons learned

## Evaluation Criteria

- **Functionality**: All components work as specified
- **Code Quality**: Well-structured, commented, and following ROS 2 conventions
- **Integration**: Components communicate properly with each other
- **Error Handling**: Proper handling of edge cases and errors
- **Documentation**: Clear comments and explanations

## Extension Ideas

1. Add a simple GUI to send commands
2. Implement a logging system for robot activities
3. Add more sophisticated navigation with path planning
4. Include sensor simulation (e.g., simulated laser scanner)
5. Add a web interface for remote control

## Troubleshooting Tips

- Ensure all nodes are on the same ROS_DOMAIN_ID
- Check that the ROS 2 environment is sourced in each terminal
- Verify topic and service names match between publishers/subscribers
- Use `ros2 doctor` to diagnose system issues
- Check firewall settings if running across multiple machines