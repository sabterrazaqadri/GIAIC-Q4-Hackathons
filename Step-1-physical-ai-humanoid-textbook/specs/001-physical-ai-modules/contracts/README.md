# API Contracts for Physical AI & Humanoid Robotics Modules

## Overview

This directory contains API contracts for the Physical AI & Humanoid Robotics textbook project. Since this project is primarily a documentation platform with educational content, traditional API contracts (REST/GraphQL) are not applicable. Instead, this directory outlines the interfaces and protocols for:

1. The simulated robot environments (Isaac Sim, Gazebo)
2. ROS 2 interfaces used in the textbook examples
3. AI service integrations (Whisper, OpenAI)

## Simulation Environment Interfaces

### Isaac Sim API References

For the Isaac Sim simulation environment used in Module 2, the textbook will reference these key interfaces:

- USD (Universal Scene Description) schemas for robot modeling
- Isaac Sim Python API for programmatically controlling simulations
- ROS 2 bridge interfaces for connecting Isaac Sim to ROS 2

### Gazebo API References

For the Gazebo simulation environment, the textbook will reference:

- Gazebo Classic/ Garden APIs for simulation control
- Gazebo plugin interfaces for custom simulation behaviors
- ROS 2 integration points for connecting Gazebo to the robotics stack

## ROS 2 Message and Service Definitions

The textbook content will reference standard ROS 2 interfaces including:

### Common Message Types
- `sensor_msgs`: Camera, LiDAR, IMU, and other sensor data
- `geometry_msgs`: Pose, Twist, Vector3, and other geometric representations
- `nav_msgs`: Path planning and navigation messages
- `std_msgs`: Standard primitive message types

### Action Interfaces
- `nav2_msgs`: Navigation actions for path planning
- `control_msgs`: Joint trajectory and gripper control actions
- Custom action definitions for humanoid-specific tasks

## AI Service Interfaces

### OpenAI API Integration

For the Vision-Language-Action (VLA) pipeline in Module 4, the textbook will outline:

- Whisper API for speech-to-text conversion
- OpenAI GPT API for language understanding and planning
- Response formats and error handling patterns

## Docusaurus Content API

The textbook itself, built with Docusaurus, uses:

- MDX format for combining Markdown with React components
- Sidebar API for navigation structure
- Theme customization interfaces
- Plugin system for extending functionality

## Educational Lab Interfaces

Each lab exercise may interact with different simulation or AI services. These interfaces will be documented in the respective chapters with:

- Expected input/output formats
- Error handling procedures
- Configuration parameters
- Troubleshooting guides

## Validation Approach

Since this is a documentation project, validation of our "contracts" will be performed by:

- Ensuring all code examples compile and execute correctly
- Verifying all simulation scenarios run in the target environments
- Confirming API calls to external services (OpenAI, etc.) are accurate
- Testing lab exercises on different hardware configurations