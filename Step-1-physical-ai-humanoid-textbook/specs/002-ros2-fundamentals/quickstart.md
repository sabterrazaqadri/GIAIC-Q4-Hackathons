# Quickstart: ROS 2 Fundamentals for Humanoid Robotics

**Feature**: 002-ros2-fundamentals | **Date**: December 16, 2025

## Overview

This quickstart guide will help you set up your environment and begin learning ROS 2 fundamentals for humanoid robotics. The module teaches ROS 2 communication models, connecting AI agents to ROS 2 controllers, and working with URDF files.

## Prerequisites

Before starting this module, ensure you have:

1. **Ubuntu 20.04 or 22.04** (recommended for compatibility with ROS 2 Humble)
2. **Basic Python knowledge** (functions, classes, modules)
3. **Familiarity with command line** operations
4. **Git** installed for version control

## Environment Setup

### Install ROS 2 Humble Hawksbill

Follow the official installation guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Quick installation commands:
```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 apt repository
sudo apt update
sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-rclpy

# Install colcon for building packages
sudo apt install python3-colcon-common-extensions

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Verify Installation

Test that ROS 2 is properly installed:
```bash
# Source the environment
source /opt/ros/humble/setup.bash

# Run a simple test
ros2 run demo_nodes_cpp talker
```

In another terminal, run:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see messages passing from the talker to the listener.

## Getting Started with the Module

### 1. Clone the Repository

```bash
git clone [repository-url]
cd [repository-name]
```

### 2. Navigate to Module 1

```bash
cd module-1
```

### 3. Start with Chapter 1

Begin with the first chapter to understand ROS 2 fundamentals:
```bash
# View the first chapter
cat chapter-1-ros2-communication.md
```

## Running the Examples

### Create a Workspace

```bash
# Create a workspace directory
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build
source install/setup.bash
```

### Run a Basic Publisher/Subscriber Example

```bash
# Terminal 1: Start the publisher
source ~/ros2_workspace/install/setup.bash
ros2 run your_package_name publisher_node

# Terminal 2: Start the subscriber
source ~/ros2_workspace/install/setup.bash
ros2 run your_package_name subscriber_node
```

## Key Concepts Covered

1. **ROS 2 Nodes**: Independent processes that communicate with each other
2. **Topics & Messages**: Asynchronous communication through named channels
3. **Services**: Synchronous request/response communication
4. **rclpy**: Python client library for ROS 2
5. **URDF**: Unified Robot Description Format for robot modeling

## Lab Exercises

Each chapter includes hands-on lab exercises. Start with:
```bash
# Navigate to lab examples
cd ~/module-1/examples/

# Follow the lab instructions in the chapter
# Run the provided example files
python3 publisher-example.py
python3 subscriber-example.py
```

## Troubleshooting

### Common Issues:

1. **"command not found" errors**: Make sure to source your ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **"Permission denied" errors**: Check your workspace permissions and make sure you own the files:
   ```bash
   sudo chown -R $USER:$USER ~/ros2_workspace
   ```

3. **Python import errors**: Ensure rclpy is installed:
   ```bash
   pip3 install rclpy
   ```

## Next Steps

1. Complete Chapter 1: ROS 2 Communication Fundamentals
2. Try the hands-on lab exercises
3. Move on to Chapter 2: Connecting AI Agents to ROS 2 Controllers
4. Practice with URDF examples in Chapter 3