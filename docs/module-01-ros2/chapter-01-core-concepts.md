---
sidebar_position: 2
---

# Chapter 1: ROS 2 Core Concepts

## Introduction

The Robot Operating System 2 (ROS 2) is not an operating system but rather a middleware framework that provides libraries, tools, and conventions for building robotic applications. It enables communication between different software components running on potentially different machines, making it easier to develop complex robotic systems.

## Architecture Overview

ROS 2 uses a distributed architecture where different components called "nodes" communicate with each other through messages. The communication happens over "topics" using a publish-subscribe pattern, or through "services" and "actions" for request-response interactions.

### Nodes

A node is a process that performs computation. In ROS 2, nodes are designed to be modular and focused on specific tasks. For example, one node might handle sensor data processing, another might handle path planning, and yet another might handle motor control.

Nodes are implemented using client libraries like:
- `rclpy` for Python
- `rclcpp` for C++
- Other language-specific libraries

### Topics and Messages

Topics are named buses over which nodes exchange messages. Messages are the data structures that carry information between nodes. The publish-subscribe communication pattern allows multiple nodes to publish data to the same topic and multiple nodes to subscribe to the same topic.

### Services

Services provide a request-response communication pattern. A client sends a request to a service server, which processes the request and sends back a response. This is useful for operations that have a clear start and end.

### Actions

Actions are used for long-running tasks that might take a significant amount of time to complete. They provide feedback during execution and can be preempted if needed. Actions are useful for navigation tasks, manipulation tasks, and other operations that take time to complete.

## DDS Communication Layer

ROS 2 uses Data Distribution Service (DDS) as its underlying communication middleware. DDS provides:

- **Reliable communication**: Messages are delivered reliably between nodes
- **Quality of Service (QoS)**: Configurable policies for message delivery, reliability, and durability
- **Discovery**: Automatic discovery of nodes on the network
- **Security**: Built-in security features for authenticated and encrypted communication

## Quality of Service (QoS) Profiles

QoS profiles allow you to configure how messages are delivered between nodes:

- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient-local (history of messages)
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to determine if a publisher is alive

## Practical Example: Understanding the Communication Model

Let's consider a simple robot with the following nodes:

1. **Sensor Node**: Publishes sensor data (e.g., laser scans) to a topic
2. **Perception Node**: Subscribes to sensor data, processes it, and publishes obstacles
3. **Planning Node**: Subscribes to obstacles, plans a path, and uses a service to execute
4. **Motion Node**: Provides a service for executing planned paths

This architecture allows each component to be developed and tested independently while maintaining clear communication interfaces.

## Security in ROS 2

ROS 2 includes security features based on DDS Security specification:

- **Authentication**: Verify the identity of nodes
- **Access Control**: Control which nodes can communicate with each other
- **Encryption**: Encrypt data in transit and at rest

## Summary

In this chapter, we've covered the fundamental concepts of ROS 2 architecture. You now understand:

- The distributed nature of ROS 2 with nodes as computation processes
- Communication patterns: topics (pub/sub), services (req/resp), and actions (long-running tasks)
- The role of DDS as the underlying communication middleware
- QoS profiles for configuring communication behavior
- Security features in ROS 2

In the next chapter, we'll implement our first ROS 2 nodes using Python and the rclpy library.