---
sidebar_position: 4
---

# Chapter 3: ROS 2 Services and Actions

## Introduction

In this chapter, we'll explore two additional communication patterns in ROS 2: services for request-response interactions and actions for long-running tasks with feedback. These patterns complement the publish-subscribe model we covered in the previous chapter.

## Services in ROS 2

Services provide a request-response communication pattern where a client sends a request to a service server, which processes the request and sends back a response. This is synchronous and blocking until the response is received.

### Creating a Service Server

Here's how to create a service server in Python:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

Here's how to create a service client that calls the service:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info('Result of add_two_ints: %d' % response.sum)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions in ROS 2

Actions are used for long-running tasks that might take a significant amount of time to complete. They provide feedback during execution and can be preempted if needed. Actions are useful for navigation tasks, manipulation tasks, and other operations that take time to complete.

### Action Structure

An action has three message types:
- **Goal**: The request sent to the action server
- **Feedback**: Messages sent periodically during execution
- **Result**: The final response after completion

### Creating an Action Server

Here's how to create an action server:

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.sequence))

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Returning result: {0}'.format(result.sequence))

        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating an Action Client

Here's how to create an action client:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.sequence))

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Common Service Types

ROS 2 comes with standard service types in packages like `std_srvs` and `example_interfaces`:

- `std_srvs/Empty`: Simple service with no request or response
- `std_srvs/SetBool`: Service for setting boolean values
- `std_srvs/Trigger`: Service that returns success/failure
- `example_interfaces/AddTwoInts`: Example service for adding two integers

## When to Use Each Communication Pattern

### Use Topics (Publish-Subscribe) When:
- Data is continuously generated (sensor data, robot state)
- Multiple subscribers need the same data
- No direct response is needed
- Real-time performance is important

### Use Services When:
- Request-response pattern is needed
- Operation has a clear beginning and end
- Client needs to wait for a result
- Operation is relatively fast (less than a few seconds)

### Use Actions When:
- Task takes a long time to complete
- Feedback during execution is needed
- Task can be preempted or canceled
- Operation involves multiple steps

## Practical Example: Robot Navigation System

Let's create a practical example combining services and actions:

**Navigation Service** (for simple movements):
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from geometry_msgs.msg import Twist

class NavigationService(Node):
    def __init__(self):
        super().__init__('navigation_service')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(
            Trigger,
            'move_forward',
            self.move_forward_callback)

    def move_forward_callback(self, request, response):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        self.cmd_vel_pub.publish(msg)

        # Move for 1 second
        self.get_clock().sleep_for(Duration(seconds=1))

        # Stop
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)

        response.success = True
        response.message = "Robot moved forward successfully"
        return response
```

**Navigation Action** (for complex navigation):
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.action import NavigateToPose

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        target_pose = goal_handle.request.pose
        self.get_logger().info(f'Navigating to pose: {target_pose}')

        # Simulate navigation with feedback
        feedback_msg = NavigateToPose.Feedback()
        for i in range(10):  # Simulate 10 steps to destination
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()

            # Update feedback
            feedback_msg.current_pose = self.get_current_pose()  # Simplified
            goal_handle.publish_feedback(feedback_msg)

            # Simulate moving toward goal
            self.get_clock().sleep_for(Duration(seconds=0.5))

        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.result = True
        return result
```

## Best Practices

1. **Service Design**: Keep service operations relatively fast; use actions for long-running tasks
2. **Error Handling**: Always handle service call failures and timeout scenarios
3. **Action Goals**: Design action goals to be specific and measurable
4. **Feedback Frequency**: Don't publish feedback too frequently; consider network overhead
5. **Cancellation**: Implement proper cancellation handling in action servers

## Summary

In this chapter, we've covered:

- How to create and use ROS 2 services for request-response communication
- How to create and use ROS 2 actions for long-running tasks with feedback
- When to use topics vs services vs actions
- Standard service and action types available in ROS 2
- Best practices for service and action design
- Practical examples combining different communication patterns

With this knowledge, you now understand all three main communication patterns in ROS 2. In the next chapter, we'll explore URDF (Unified Robot Description Format) for describing robot models.