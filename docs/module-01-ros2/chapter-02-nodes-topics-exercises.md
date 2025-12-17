---
sidebar_position: 5
---

# Chapter 2: Exercises - Nodes and Topics

## Exercise 2.1: Basic Publisher Node

### Objective
Create a simple publisher node that publishes a custom message to a topic.

### Tasks
1. Create a new Python file called `simple_publisher.py`
2. Implement a ROS 2 node that publishes the string "Hello ROS 2" every 2 seconds
3. Use the `std_msgs.msg.String` message type
4. Publish to a topic called `my_topic`
5. Run your node and verify it's publishing using `ros2 topic echo`

### Code Template
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        # TODO: Create publisher
        # TODO: Create timer

    def timer_callback(self):
        # TODO: Create and publish message
        pass

def main(args=None):
    # TODO: Initialize ROS, create node, and spin
    pass
```

### Questions
1. What is the purpose of the `depth` parameter when creating a publisher?
2. What happens if you don't call `rclpy.spin()`?
3. How would you modify the code to publish different messages each time?

### Expected Outcome
Your node should successfully publish messages that can be received by other nodes or viewed with command-line tools.

## Exercise 2.2: Basic Subscriber Node

### Objective
Create a subscriber node that receives messages from the publisher in Exercise 2.1.

### Tasks
1. Create a new Python file called `simple_subscriber.py`
2. Implement a ROS 2 node that subscribes to the `my_topic` topic
3. Print received messages to the console
4. Run both publisher and subscriber nodes to see the communication

### Code Template
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        # TODO: Create subscription

    def subscription_callback(self, msg):
        # TODO: Process received message
        pass

def main(args=None):
    # TODO: Initialize ROS, create node, and spin
    pass
```

### Questions
1. What is the purpose of the callback function in a subscriber?
2. Can multiple subscribers listen to the same topic?
3. What happens if the subscriber starts after the publisher?

### Expected Outcome
Your subscriber should successfully receive and process messages from the publisher.

## Exercise 2.3: Custom Message Publisher

### Objective
Create a publisher that sends custom messages with multiple data types.

### Tasks
1. Create a publisher that sends messages containing:
   - A timestamp (float)
   - A sensor reading (float)
   - A status flag (boolean)
2. Use the `std_msgs.msg.Float64MultiArray` or create a custom approach using multiple standard messages
3. Simulate sensor readings with random values
4. Verify the messages are being published correctly

### Questions
1. How would you create a truly custom message type?
2. What are the advantages of using standard message types vs custom ones?
3. How would you ensure data consistency across multiple fields?

### Expected Outcome
You should be able to publish complex data structures using standard ROS 2 message types.

## Exercise 2.4: Multiple Publishers and Subscribers

### Objective
Create a system with multiple publishers and subscribers on the same topic.

### Tasks
1. Create two publisher nodes that publish to the same topic with different content
2. Create two subscriber nodes that subscribe to the same topic
3. Observe how messages are distributed among subscribers
4. Add timestamps to messages to observe ordering

### Questions
1. Do all subscribers receive messages from all publishers?
2. How are messages ordered when multiple publishers send to the same topic?
3. What happens if one publisher sends messages much faster than another?

### Expected Outcome
You should understand how ROS 2 handles multiple publishers and subscribers on the same topic.

## Exercise 2.5: Quality of Service Experimentation

### Objective
Experiment with different QoS settings and observe their effects.

### Tasks
1. Create a publisher with `reliability=RELIABLE` and `durability=VOLATILE`
2. Create a subscriber with the same QoS settings
3. Test communication performance
4. Change to `reliability=BEST_EFFORT` and observe differences
5. Test with `durability=TRANSIENT_LOCAL` and late-joining subscribers

### Code Template
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Example QoS profile
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

### Questions
1. What is the difference between RELIABLE and BEST_EFFORT?
2. When would you use TRANSIENT_LOCAL durability?
3. How do QoS settings affect system performance?

### Expected Outcome
You should understand the impact of different QoS settings on communication behavior.

## Exercise 2.6: Topic Monitoring and Analysis

### Objective
Use ROS 2 command-line tools to monitor and analyze topic communication.

### Tasks
1. Create a publisher node that publishes messages at different rates
2. Use `ros2 topic hz` to measure message frequency
3. Use `ros2 topic bw` to measure bandwidth
4. Use `ros2 topic delay` to measure message delay
5. Analyze the results and optimize your node if needed

### Commands to Try
```bash
# Measure frequency
ros2 topic hz /your_topic

# Measure bandwidth
ros2 topic bw /your_topic

# Monitor messages
ros2 topic echo /your_topic
```

### Questions
1. How does actual frequency compare to desired frequency?
2. What factors might affect message delay?
3. How can you optimize your node for better performance?

### Expected Outcome
You should be able to monitor and analyze ROS 2 topic performance effectively.

## Exercise 2.7: Launch File Integration

### Objective
Create a launch file to run multiple nodes together.

### Tasks
1. Create a Python launch file that starts your publisher and subscriber nodes
2. Add parameters to control node behavior
3. Test the launch file to ensure both nodes start correctly
4. Add error handling for when nodes fail to start

### Code Template
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='simple_publisher',
            name='publisher_node',
            parameters=[
                # Add parameters here
            ]
        ),
        Node(
            package='your_package',
            executable='simple_subscriber',
            name='subscriber_node',
        ),
    ])
```

### Questions
1. What are the advantages of using launch files?
2. How do you pass parameters to nodes in launch files?
3. Can you start multiple instances of the same node?

### Expected Outcome
You should be able to use launch files to manage complex ROS 2 systems.

## Solutions

### Exercise 2.1 Solution
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.get_clock().now().nanoseconds
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 2.2 Solution
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.subscription_callback,
            10)
        self.subscription  # prevent unused variable warning

    def subscription_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 2.3 Solution Approach
For complex data, you can use multiple approaches:
1. Use `std_msgs.msg.Float64MultiArray` to send multiple float values
2. Create a custom message type (requires package setup)
3. Use multiple topics for different data components
4. Use a string message with formatted data (not recommended for production)