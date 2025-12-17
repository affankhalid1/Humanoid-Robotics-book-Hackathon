#!/usr/bin/env python3

"""
Simple Publisher Example

This node publishes messages to a topic called 'robot_status'.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')

        # Create a publisher for the 'robot_status' topic
        self.publisher = self.create_publisher(String, 'robot_status', 10)

        # Create a timer to publish messages periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for the messages
        self.counter = 0

        self.get_logger().info('Robot Status Publisher node initialized')

    def timer_callback(self):
        # Create a message
        msg = String()
        self.counter += 1
        msg.data = f'Robot is operational - Message #{self.counter}'

        # Publish the message
        self.publisher.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of our node
    publisher_node = RobotStatusPublisher()

    # Keep the node running until it's shut down
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()