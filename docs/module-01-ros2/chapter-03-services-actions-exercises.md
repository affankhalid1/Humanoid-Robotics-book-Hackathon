---
sidebar_position: 5
---

# Chapter 3: Exercises - Services and Actions

## Exercise 3.1: Basic Service Server

### Objective
Create a simple service server that performs a calculation.

### Tasks
1. Create a service server that adds two integers together
2. Use the `example_interfaces/srv/AddTwoInts` service type
3. Implement the service callback to return the sum
4. Test the service using the command-line client

### Code Template
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleService(Node):
    def __init__(self):
        super().__init__('simple_service')
        # TODO: Create service server

    def add_callback(self, request, response):
        # TODO: Implement addition logic
        return response

def main(args=None):
    # TODO: Initialize ROS, create node, and spin
    pass
```

### Questions
1. What is the difference between request and response in a service?
2. How does the service handle multiple simultaneous requests?
3. What happens if the service callback takes too long to execute?

### Expected Outcome
Your service should successfully respond to requests with the correct sum of two numbers.

## Exercise 3.2: Basic Service Client

### Objective
Create a client that calls the service from Exercise 3.1.

### Tasks
1. Create a service client that connects to the AddTwoInts service
2. Send a request with two numbers (e.g., 5 and 3)
3. Wait for and display the response
4. Handle potential errors in service communication

### Code Template
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleClient(Node):
    def __init__(self):
        super().__init__('simple_client')
        # TODO: Create service client
        # TODO: Wait for service to be available

    def send_request(self, a, b):
        # TODO: Send request and return result
        pass

def main(args=None):
    # TODO: Initialize ROS, create client, send request, and shutdown
    pass
```

### Questions
1. What happens if the service is not available when the client starts?
2. How does the client know when the service call is complete?
3. What are the advantages of using services over topics for this use case?

### Expected Outcome
Your client should successfully call the service and receive the correct response.

## Exercise 3.3: Custom Service

### Objective
Create a custom service that performs a more complex operation.

### Tasks
1. Define a custom service that calculates the factorial of a number
2. The service should take a single integer as input and return the factorial
3. Include error handling for negative numbers
4. Test both valid and invalid inputs

### Service Definition (Conceptual)
- Request: `int64 input_number`
- Response: `int64 result`, `bool success`, `string error_message`

### Questions
1. How would you define a custom service type?
2. What are the considerations for handling large numbers in factorials?
3. When should you return an error vs a special value?

### Expected Outcome
You should understand how to design and implement custom services for specific operations.

## Exercise 3.4: Basic Action Server

### Objective
Create a simple action server that simulates a long-running task.

### Tasks
1. Create an action server that simulates a robot moving to a goal position
2. Use the action to move from current position to a target position
3. Provide feedback on the progress of the movement
4. Handle cancellation requests

### Code Template
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
# TODO: Import appropriate action type (e.g., from example_interfaces.action import Fibonacci)

class MoveRobotActionServer(Node):
    def __init__(self):
        super().__init__('move_robot_action_server')
        # TODO: Create action server with execute callback

    async def execute_callback(self, goal_handle):
        # TODO: Implement the long-running task with feedback
        pass

def main(args=None):
    # TODO: Initialize ROS, create node, and spin
    pass
```

### Questions
1. How is an action different from a service for long-running tasks?
2. What happens when a goal is cancelled?
3. How does feedback differ from just publishing status?

### Expected Outcome
Your action server should handle long-running tasks with progress feedback and cancellation support.

## Exercise 3.5: Action Client

### Objective
Create a client that sends goals to the action server from Exercise 3.4.

### Tasks
1. Create an action client that sends movement goals
2. Handle feedback messages during execution
3. Handle the final result when the action completes
4. Implement a timeout for the action

### Code Template
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
# TODO: Import appropriate action type

class MoveRobotActionClient(Node):
    def __init__(self):
        super().__init__('move_robot_action_client')
        # TODO: Create action client

    def send_goal(self, target_position):
        # TODO: Send goal and handle response
        pass

def main(args=None):
    # TODO: Initialize ROS, create client, send goal, and spin
    pass
```

### Questions
1. How does the action client handle feedback differently from a service client?
2. What happens if you send a new goal while another is executing?
3. How do you cancel a goal from the client side?

### Expected Outcome
Your action client should successfully communicate with the server, receive feedback, and handle the final result.

## Exercise 3.6: Service vs Action Comparison

### Objective
Compare the use of services and actions for different scenarios.

### Tasks
1. Implement the same functionality using both a service and an action:
   - Calculating a path through a map
   - The service returns immediately with the path
   - The action provides feedback during path calculation
2. Compare the implementation complexity
3. Test both approaches and note the differences in behavior

### Questions
1. When is a service more appropriate than an action?
2. When is an action more appropriate than a service?
3. How do error handling and timeouts differ between the two?

### Expected Outcome
You should understand when to use services vs actions based on the nature of the task.

## Exercise 3.7: Real-world Scenario Implementation

### Objective
Implement a realistic scenario using both services and actions.

### Scenario
A robot cleaning system with:
- Service: Request cleaning of a specific room
- Action: Execute the actual cleaning process (long-running with feedback)

### Tasks
1. Create a cleaning request service (fast response)
2. Create a cleaning execution action (long-running with feedback)
3. Implement both server and client
4. Test the complete workflow

### Questions
1. Why use both a service and an action for this scenario?
2. How would you handle errors in the cleaning process?
3. What feedback would be most useful during cleaning?

### Expected Outcome
You should be able to design a system that appropriately uses both services and actions.

## Exercise 3.8: Error Handling and Robustness

### Objective
Implement proper error handling in services and actions.

### Tasks
1. Add timeout handling to your service client
2. Add proper error responses to your service server
3. Implement goal acceptance/rejection logic in your action server
4. Add preemption handling to your action server

### Questions
1. How should services handle computation-intensive operations?
2. What is the difference between rejecting a goal and accepting it but failing?
3. How do you ensure actions can be safely cancelled?

### Expected Outcome
You should implement robust error handling for both services and actions.

## Solutions

### Exercise 3.1 Solution
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleService(Node):
    def __init__(self):
        super().__init__('simple_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service = SimpleService()
    rclpy.spin(simple_service)
    simple_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 3.2 Solution
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleClient(Node):
    def __init__(self):
        super().__init__('simple_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    simple_client = SimpleClient()
    response = simple_client.send_request(5, 3)
    simple_client.get_logger().info(f'Result: {response.sum}')
    simple_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 3.4 Solution (Simplified Fibonacci Action)
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```