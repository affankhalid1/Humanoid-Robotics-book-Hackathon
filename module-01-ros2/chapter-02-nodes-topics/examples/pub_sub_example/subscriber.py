#!/usr/bin/env python3

"""
Simple Subscriber Example

This node subscribes to messages from the 'robot_status' topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotStatusSubscriber(Node):
    def __init__(self):
        super().__init__('robot_status_subscriber')

        # Create a subscription to the 'robot_status' topic
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10  # QoS profile depth
        )

        # Prevent unused variable warning
        self.subscription  # No need to assign to self unless used elsewhere

        self.get_logger().info('Robot Status Subscriber node initialized')

    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info(f'Received robot status: "{msg.data}"')


def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of our node
    subscriber_node = RobotStatusSubscriber()

    # Keep the node running until it's shut down
    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()