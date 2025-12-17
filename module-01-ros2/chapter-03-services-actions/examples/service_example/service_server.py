#!/usr/bin/env python3

"""
Simple Service Server Example

This node provides a service to add two numbers together.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_service')

        # Create a service server
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Add Two Ints Service server initialized')

    def add_two_ints_callback(self, request, response):
        # Perform the addition
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(
            f'Request received: {request.a} + {request.b} = {response.sum}'
        )

        return response


def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of our node
    service_node = AddTwoIntsService()

    # Keep the node running until it's shut down
    try:
        rclpy.spin(service_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()