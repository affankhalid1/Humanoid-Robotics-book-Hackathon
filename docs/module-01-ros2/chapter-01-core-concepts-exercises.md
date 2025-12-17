---
sidebar_position: 5
---

# Chapter 1: Exercises - Core Concepts

## Exercise 1.1: Node Discovery and Communication

### Objective
Understand how ROS 2 nodes discover each other and communicate using the command line tools.

### Tasks
1. Open two terminal windows
2. In the first terminal, source your ROS 2 environment: `source /opt/ros/humble/setup.bash`
3. Run a simple publisher node: `ros2 run demo_nodes_cpp talker`
4. In the second terminal, source the ROS 2 environment
5. List all active nodes: `ros2 node list`
6. List all active topics: `ros2 topic list`
7. Echo the messages being published: `ros2 topic echo /chatter std_msgs/msg/String`

### Questions
1. How many nodes do you see when running `ros2 node list`?
2. What is the type of the `/chatter` topic?
3. What happens when you stop the talker node?

### Expected Outcome
You should be able to see the communication between nodes and understand how ROS 2 enables node discovery.

## Exercise 1.2: Quality of Service (QoS) Settings

### Objective
Explore different Quality of Service settings and their impact on communication.

### Tasks
1. Run a talker node with specific QoS settings:
   ```bash
   ros2 run demo_nodes_cpp talker --ros-args -p use_default_qos:=false -p reliability:=best_effort
   ```
2. In another terminal, run a listener with compatible QoS:
   ```bash
   ros2 run demo_nodes_cpp listener --ros-args -p use_default_qos:=false -p reliability:=best_effort
   ```
3. Observe the communication behavior
4. Try with incompatible QoS settings (reliability mismatch) and observe the difference

### Questions
1. What happens when QoS settings don't match between publisher and subscriber?
2. When would you use "best effort" vs "reliable" QoS?
3. What is the difference between "volatile" and "transient_local" durability?

### Expected Outcome
You should understand how QoS settings affect communication reliability and performance.

## Exercise 1.3: DDS Discovery Process

### Objective
Understand how DDS enables automatic discovery of nodes in the ROS 2 network.

### Tasks
1. Set up two different ROS_DOMAIN_IDs in separate terminals:
   ```bash
   # Terminal 1
   export ROS_DOMAIN_ID=1
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_cpp talker
   ```
   ```bash
   # Terminal 2
   export ROS_DOMAIN_ID=2
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_cpp listener
   ```
2. List nodes in each domain separately
3. Try with the same domain ID and observe the difference

### Questions
1. Can nodes in different domains communicate with each other?
2. Why might you want to use different domain IDs?
3. What is the default domain ID?

### Expected Outcome
You should understand domain isolation and how it can be used to separate different ROS 2 networks.

## Exercise 1.4: Service Discovery

### Objective
Explore how ROS 2 services are discovered and used.

### Tasks
1. Run a service server (if available in your setup)
2. List available services: `ros2 service list`
3. Check the type of a service: `ros2 service type <service_name>`
4. Call a service from the command line: `ros2 service call <service_name> <service_type> <request>`

### Questions
1. How are services different from topics in terms of discovery?
2. What information does `ros2 service info` provide?
3. When would you use a service instead of a topic?

### Expected Outcome
You should understand service-based communication and how it differs from topic-based communication.

## Exercise 1.5: System Architecture Analysis

### Objective
Analyze a simple robotic system architecture using ROS 2 concepts.

### Scenario
Consider a simple robot with:
- Camera node (publishes images)
- Object detection node (subscribes to images, publishes detected objects)
- Navigation node (subscribes to detected objects, navigates to objects)
- UI node (subscribes to robot status, displays on screen)

### Tasks
1. Draw a node graph showing how these nodes would communicate
2. Identify what topics would be used and their message types
3. Identify where services might be useful in this system
4. Consider what QoS settings would be appropriate for each communication link

### Questions
1. Which communication pattern would be best for camera images?
2. What QoS settings would you use for safety-critical navigation commands?
3. How would you handle the case where the object detection node fails?

### Expected Outcome
You should be able to design a basic ROS 2 system architecture for a simple robot.

## Solutions

### Exercise 1.1 Solutions
1. You should see at least 2 nodes: the talker node and the parameter_services_node for the listener
2. The type should be `std_msgs/msg/String`
3. The listener will stop receiving messages when the talker stops

### Exercise 1.2 Solutions
1. When QoS settings don't match, communication may be unreliable or fail completely
2. Use "best effort" for non-critical data like camera feeds; "reliable" for critical commands
3. "Volatile" doesn't keep historical data; "transient_local" keeps last message for late-joining subscribers

### Exercise 1.3 Solutions
1. No, nodes in different domains cannot communicate
2. To separate different robot systems or development environments
3. The default domain ID is 0

### Exercise 1.4 Solutions
1. Services are discovered through service names, not topics
2. `ros2 service info` shows node providing the service and endpoints
3. Use services for request-response operations like configuration or single computations

### Exercise 1.5 Solutions
1. For camera images, consider reliability=best_effort, durability=volatile for performance
2. For safety-critical commands, use reliability=reliable, durability=volatile with appropriate deadlines
3. Implement error handling and fallback behaviors in the navigation node