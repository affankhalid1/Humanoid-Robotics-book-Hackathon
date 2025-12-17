#!/usr/bin/env python3

"""
Simple Service Client Example

This node calls the add_two_ints service to add two numbers.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create a client for the 'add_two_ints' service
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        # Set the request parameters
        self.request.a = a
        self.request.b = b

        # Call the service asynchronously
        self.future = self.client.call_async(self.request)

        # Log the request
        self.get_logger().info(f'Sending request: {a} + {b}')


def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of our client node
    client_node = AddTwoIntsClient()

    # Check if command line arguments are provided
    if len(sys.argv) != 3:
        print('Usage: ros2 run service_example service_client <int1> <int2>')
        print('Example: ros2 run service_example service_client 2 3')
        sys.exit(1)

    # Parse the command line arguments
    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('Please provide two integer arguments')
        sys.exit(1)

    # Send the request
    client_node.send_request(a, b)

    # Wait for the response
    try:
        while rclpy.ok():
            rclpy.spin_once(client_node)
            if client_node.future.done():
                try:
                    response = client_node.future.result()
                    print(f'Result: {a} + {b} = {response.sum}')
                except Exception as e:
                    client_node.get_logger().error(f'Service call failed: {e}')
                break
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()