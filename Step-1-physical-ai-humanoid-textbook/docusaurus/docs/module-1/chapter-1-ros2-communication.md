# Chapter 1: ROS 2 Communication Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the core concepts of ROS 2 as a "robotic nervous system"
- Identify and differentiate between Nodes, Topics, and Services
- Describe how messages are passed between different components
- Understand the publish-subscribe and client-server communication patterns

## Introduction

Welcome to the fundamentals of ROS 2! Just like the human nervous system allows different parts of our body to communicate and coordinate, ROS 2 (Robot Operating System 2) enables different parts of a robot to communicate effectively.

ROS 2 is not an actual operating system but rather a flexible framework that provides the tools, libraries, and conventions needed to build robot applications. It handles the "plumbing" of robotic applications, allowing you to focus on high-level functionality.

In this chapter, we'll explore the foundational concepts that make ROS 2 the "robotic nervous system."

## Main Content

### Introduction to ROS 2 Architecture

![ROS 2 Architecture Diagram](pathname:// "ROS 2 Architecture Diagram showing nodes, topics, and services")

The above diagram illustrates the core architecture of ROS 2. As you can see, different nodes communicate via topics using a publish-subscribe pattern, and via services using a request-response pattern.

### What is ROS 2?

ROS 2 is the second generation of the Robot Operating System framework. It provides:

- A communication layer for different parts of your robot to interact
- Tools for debugging, visualization, and testing
- Hardware abstraction for various sensors and actuators
- Package management for reusable code

ROS 2 is designed to be more robust, secure, and suitable for commercial applications compared to its predecessor, ROS 1.

### The ROS 2 Ecosystem

In ROS 2, different parts of your robot application are organized as **Nodes**. Think of nodes as specialized organs that perform specific functions. Nodes communicate with each other through:

1. **Topics** - for asynchronous, one-way communication (publish-subscribe pattern)
2. **Services** - for synchronous, request-response communication (client-server pattern)

This architecture allows for a distributed system where multiple computers can work together as a unified robot system.

### Nodes: The Building Blocks

A **Node** is the fundamental unit of computation in ROS 2. Nodes are processes that perform specific tasks, such as:

- Reading sensor data (e.g., camera, LIDAR)
- Controlling motors or actuators
- Processing data and making decisions
- Displaying information to users

Nodes are often written in C++ or Python, though ROS 2 supports other languages too.

#### Creating a Node

In code, a node is typically a class that inherits from the rclpy.Node base class in Python or rclcpp::Node in C++. Each node has a unique name within the ROS 2 graph and can contain publishers, subscribers, services, clients, and other ROS 2 entities.

### Topics: One-Way Communication

**Topics** enable asynchronous communication using the publish-subscribe pattern. Publishers send data to a topic, and subscribers receive data from that topic. This decouples the sender and receiver, allowing for flexible system design.

Key characteristics:
- Multiple publishers can send to the same topic
- Multiple subscribers can listen to the same topic
- No direct connection between publisher and subscriber
- Asynchronous: publisher sends messages without waiting for acknowledgment

#### Topic Names

Topic names follow a hierarchical structure similar to file paths:
- `/cmd_vel` - commands for robot velocity
- `/sensor_data/laser_scan` - laser scanner data
- `/robot/manipulator/joint_states` - joint position information

### Services: Two-Way Communication

**Services** provide synchronous, request-response communication. A client sends a request to a service server, which processes the request and returns a response. This is ideal for tasks that require confirmation or specific responses.

Key characteristics:
- One client communicates with one server at a time
- Synchronous: client waits for the response
- Request-response pattern with specific message types for request and response

#### Common Service Usage

- Parameter configuration
- Navigation goal setting
- Data retrieval from specific sensors
- System activation/deactivation

### Quality of Service (QoS)

ROS 2 uses Quality of Service profiles to specify communication requirements:
- **Reliability**: Reliable (all messages delivered) or best effort (try to deliver)
- **Durability**: Volatile (only new messages) or transient local (historical messages)
- **History**: Keep all messages or only the last N messages

These settings help optimize communication for different use cases, from real-time control (best effort) to persistent data (reliable, durable).

## Hands-on Lab

### Lab Objective
Set up a basic ROS 2 environment and create your first publisher and subscriber nodes to understand message passing.

### Prerequisites
- ROS 2 Humble Hawksbill installed
- Basic Python knowledge
- Terminal/command prompt access

### Steps
1. **Source your ROS 2 environment**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Create a new workspace**
   ```bash
   mkdir -p ~/ros2_workspace/src
   cd ~/ros2_workspace
   ```

3. **Create a simple publisher node** (already created in examples/publisher-example.py)

4. **Create a simple subscriber node** (already created in examples/subscriber-example.py)

5. **Terminal 1: Run the publisher node**
   ```bash
   cd ~/ros2_workspace
   source install/setup.bash
   python3 examples/publisher-example.py
   ```

6. **Terminal 2: Run the subscriber node**
   ```bash
   cd ~/ros2_workspace
   source install/setup.bash
   python3 examples/subscriber-example.py
   ```

7. **Observe the communication** between the two nodes via the `/chatter` topic.

### Expected Output
- Publisher should output "Hello World: count" messages every 0.5 seconds
- Subscriber should receive and display these messages with a timestamp

### Validation Commands
```bash
# Check active nodes
ros2 node list

# Check active topics
ros2 topic list

# Echo messages on the chatter topic
ros2 topic echo /chatter std_msgs/msg/String
```

### Hints
- Make sure both terminals have the ROS 2 environment sourced
- If nodes don't communicate, check that they're using the same topic name
- You can use `ros2 doctor` to diagnose common issues

## Comprehension Checks

### Question 1
What is the primary purpose of a Node in ROS 2?
A) To store robot configuration data
B) To serve as the fundamental unit of computation
C) To manage the robot's power system
D) To provide a user interface for robot control

**Answer: B** - Nodes are the fundamental units of computation in ROS 2, similar to specialized organs in the human body.

### Question 2
Which communication pattern is used by Topics in ROS 2?
A) Request-response
B) Client-server
C) Publish-subscribe
D) Peer-to-peer

**Answer: C** - Topics use the publish-subscribe pattern where publishers send data and subscribers receive data without direct connection.

### Question 3
True or False: In ROS 2, a service client waits for a response from the service server before continuing.
A) True
B) False

**Answer: A** - True. Service communication is synchronous, meaning the client waits for the server's response before continuing.

## Summary

In this chapter, we've covered the foundational concepts of ROS 2:
- Nodes serve as the fundamental building blocks of ROS 2 applications
- Topics enable asynchronous communication through publish-subscribe pattern
- Services enable synchronous communication through request-response pattern
- Quality of Service settings allow optimization for specific use cases

These concepts form the basis of all ROS 2 applications and are essential for understanding more advanced topics in subsequent chapters.

## References and Citations

- ROS 2 Documentation. (2023). *Concepts: About ROS 2*. Available at: https://docs.ros.org/en/humble/Concepts/About-ROS-2.html
- ROS 2 Design Articles. (n.d.). *ROS 2 Communication Model*. Available at: https://design.ros2.org/
- rclpy Documentation. (2023). *Python Client Library for ROS 2*. Available at: https://docs.ros.org/en/humble/p/rclpy/
- Cousins, W., & The ROS Community. (2020). *ROS 2 Design: Improving Build, Interface, and Communications*. Available at: https://roscon.ros.org/2020/