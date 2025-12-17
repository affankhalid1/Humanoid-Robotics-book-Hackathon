#!/usr/bin/env python3

"""
Hello Robot World - Basic ROS 2 Node Example

This is a simple example that demonstrates the basic structure of a ROS 2 node.
It doesn't actually communicate with other nodes but shows the essential components.
"""

import rclpy
from rclpy.node import Node


class HelloRobotWorld(Node):
    def __init__(self):
        super().__init__('hello_robot_world')
        self.get_logger().info('Hello Robot World! This is my first ROS 2 node.')

        # This counter is just for demonstration purposes
        self.counter = 0

        # Create a timer to periodically log a message
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Hello Robot World! Message #{self.counter}')


def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of our node
    hello_robot_node = HelloRobotWorld()

    # Keep the node running until it's shut down
    try:
        rclpy.spin(hello_robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        hello_robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()