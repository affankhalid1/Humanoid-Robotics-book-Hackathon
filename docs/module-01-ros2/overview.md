---
sidebar_position: 1
---

# Module 1: ROS 2 Fundamentals

## Overview

This module introduces you to the Robot Operating System 2 (ROS 2), the middleware framework that provides the communication backbone for robotic applications. ROS 2 is essential for building distributed robotic systems where multiple processes need to communicate seamlessly.

### Learning Objectives

By the end of this module, you will be able to:

- Understand the core concepts of ROS 2 architecture
- Create and run ROS 2 nodes using Python
- Implement publisher-subscriber communication patterns
- Use services and actions for request-response communication
- Describe robots using URDF (Unified Robot Description Format)

### Module Structure

This module is organized into the following chapters:

1. **Chapter 1 - Core Concepts**: Introduction to ROS 2 architecture, nodes, topics, and the DDS communication layer
2. **Chapter 2 - Python Nodes**: Creating ROS 2 nodes using the rclpy library, implementing publishers and subscribers
3. **Chapter 3 - Services and Actions**: Request-response communication patterns and long-running tasks
4. **Chapter 4 - URDF Description**: Describing robot kinematics and visual properties

### Prerequisites

Before starting this module, ensure you have:

- Basic Python programming skills
- Understanding of object-oriented programming concepts
- Familiarity with Linux command line
- Docker installed (for isolated development environment)

### Hardware Requirements

For this module, you'll need:

- Ubuntu 22.04 LTS (or Windows with WSL2/VM)
- ROS 2 Humble Hawksbill installed
- Basic development tools (Git, Python 3.10+)

### Project: Hello Robot World

At the end of this module, you'll implement the "Hello Robot World" project, where you'll create a simple robot node that publishes its status and responds to commands. This project will integrate all the concepts learned in this module.

### Next Steps

Start with Chapter 1 to learn about the fundamental concepts of ROS 2 architecture and the DDS communication layer that powers modern robotic systems.