---
sidebar_position: 3
---

# Chapter 2: ROS 2 Nodes and Topics

## Introduction

In this chapter, we'll dive into creating ROS 2 nodes using Python and implementing the publish-subscribe communication pattern. We'll cover the fundamental building blocks of ROS 2 applications and learn how to create nodes that can communicate with each other.

## Creating a ROS 2 Node in Python

To create a ROS 2 node in Python, we use the `rclpy` client library. Here's the basic structure of a ROS 2 node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publisher Nodes

A publisher node sends messages to a topic. Here's an example of a simple publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber Nodes

A subscriber node receives messages from a topic. Here's an example of a simple subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Settings

When creating publishers and subscribers, you can specify QoS settings to control how messages are delivered:

```python
from rclpy.qos import QoSProfile

# Create a QoS profile with reliability and durability settings
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Use the QoS profile when creating publisher/subscriber
self.publisher_ = self.create_publisher(String, 'topic', qos_profile)
```

## Message Types

ROS 2 comes with standard message types in packages like `std_msgs`, `geometry_msgs`, and `sensor_msgs`. You can also define custom message types. Common message types include:

- `std_msgs/String`: Simple string messages
- `std_msgs/Int32`: Integer values
- `geometry_msgs/Twist`: Velocity commands for robots
- `sensor_msgs/LaserScan`: Laser range finder data
- `sensor_msgs/Image`: Image data from cameras

## Creating Custom Messages

To create a custom message, you define it in a `.msg` file in the `msg/` directory of your package:

```
# MyCustomMessage.msg
string name
int32 id
float64[] values
```

Then add it to your `CMakeLists.txt` and `package.xml` to generate the message definitions.

## Launch Files

Launch files allow you to start multiple nodes with a single command. Here's a simple launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='publisher_node',
            name='publisher',
        ),
        Node(
            package='my_package',
            executable='subscriber_node',
            name='subscriber',
        ),
    ])
```

## Practical Example: Temperature Monitor

Let's create a practical example with a temperature sensor publisher and a temperature monitor subscriber:

**Temperature Publisher:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        # Simulate temperature reading (20-30 degrees)
        msg.data = 20.0 + random.uniform(0, 10)
        self.publisher_.publish(msg)
        self.get_logger().info('Temperature: %.2f' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    temp_publisher = TemperaturePublisher()
    rclpy.spin(temp_publisher)
    temp_publisher.destroy_node()
    rclpy.shutdown()
```

**Temperature Monitor:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10)
        self.subscription  # prevent unused variable warning

    def temperature_callback(self, msg):
        if msg.data > 25.0:
            self.get_logger().warn('High temperature detected: %.2f' % msg.data)
        else:
            self.get_logger().info('Temperature: %.2f' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    temp_monitor = TemperatureMonitor()
    rclpy.spin(temp_monitor)
    temp_monitor.destroy_node()
    rclpy.shutdown()
```

## Best Practices

1. **Node Naming**: Use descriptive names for nodes that clearly indicate their purpose
2. **Topic Naming**: Use consistent naming conventions (e.g., `/robot_name/sensor_type`)
3. **QoS Settings**: Choose appropriate QoS settings based on your application requirements
4. **Error Handling**: Implement proper error handling and logging in your nodes
5. **Resource Management**: Always properly clean up resources when nodes are destroyed

## Debugging ROS 2 Nodes

Useful tools for debugging ROS 2 nodes include:

- `ros2 topic list`: List all available topics
- `ros2 topic echo <topic_name>`: Print messages from a topic
- `ros2 node list`: List all active nodes
- `ros2 run <package_name> <executable_name>`: Run a node directly

## Summary

In this chapter, we've covered:

- How to create ROS 2 nodes using Python and rclpy
- Implementing publisher and subscriber nodes
- Using Quality of Service (QoS) settings
- Working with standard and custom message types
- Creating launch files to run multiple nodes
- Best practices for node development
- Debugging tools for ROS 2 applications

In the next chapter, we'll explore services and actions for request-response communication patterns.