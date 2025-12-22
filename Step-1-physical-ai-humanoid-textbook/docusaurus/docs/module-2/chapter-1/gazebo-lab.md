---
title: Lab Exercise - Gazebo Basic Environment Setup
sidebar_position: 3
---

# Lab Exercise: Gazebo Basic Environment Setup

## Learning Objectives

- Install and configure Gazebo for robotics simulation
- Create a basic simulation environment in Gazebo
- Understand the fundamental components of Gazebo
- Verify that Gazebo is running correctly with ROS 2 integration

## Prerequisites

- Ubuntu 22.04 LTS or compatible system
- ROS 2 Humble Hawksbill installed
- Basic familiarity with Linux command line
- Basic understanding of robotics concepts

## Setup Instructions

1. **Install ROS 2 Humble**: If not already installed, follow the official ROS 2 installation guide for Ubuntu 22.04.

2. **Install Gazebo Garden**: Install the Gazebo Garden simulation environment which is compatible with ROS 2 Humble.

3. **Verify Installation**: Ensure ROS 2 is properly sourced and Gazebo can be launched.

## Procedure

### Step 1: Verify ROS 2 Installation
- **Commands:**
  ```
  # Check ROS 2 installation
  ros2 --version
  
  # Source ROS 2 (if needed)
  source /opt/ros/humble/setup.bash
  ```
- **Expected Result:** ROS 2 version should be displayed (e.g., "ros2 foxy", "ros2 humble")

### Step 2: Install Gazebo Garden
- **Commands:**
  ```
  # Install Gazebo Garden
  sudo apt-get update
  sudo apt-get install gz-fortress
  
  # Also install ROS 2 Gazebo packages
  sudo apt-get install ros-humble-gazebo-*
  ```
- **Expected Result:** Gazebo Garden and ROS 2 Gazebo packages are installed successfully

### Step 3: Launch Gazebo
- **Commands:**
  ```
  # Launch Gazebo with an empty world
  gazebo
  
  # Or using gz command
  gz sim
  ```
- **Expected Result:** Gazebo GUI opens with the default empty world

### Step 4: Create a New World (Optional)
- **Commands:**
  ```
  # In Gazebo GUI
  1. Go to File > Save World As
  2. Name your world (e.g., "my_first_world.sdf")
  3. Save it in a convenient location
  ```
- **Expected Result:** A new world file is created with the current scene configuration

### Step 5: Add Objects to the World
- **Commands:**
  ```
  # In Gazebo GUI
  1. Go to the "Insert" tab
  2. Drag and drop a model (e.g., a cube or sphere) into the world
  3. Position the model using the translation tool
  ```
- **Expected Result:** The selected model appears in the simulation world

### Step 6: Start Simulation
- **Commands:**
  ```
  # In Gazebo GUI
  1. Click the "Play" button (>) at the bottom of the GUI
  2. Observe the physics simulation
  3. Click "Pause" (||) to stop the simulation
  ```
- **Expected Result:** Physics simulation starts, and objects behave according to physical laws (e.g., gravity)

### Step 7: Test ROS Integration
- **Commands:**
  ```
  # Terminal 1: Launch Gazebo (if not already running)
  gazebo
  
  # Terminal 2: Check available ROS topics
  source /opt/ros/humble/setup.bash
  ros2 topic list
  
  # Terminal 2: Echo simulation time
  ros2 topic echo /clock
  ```
- **Expected Result:** ROS topics related to Gazebo simulation are available, and clock messages are published

## Expected Output

After completing this lab, you should have:

1. Gazebo running correctly with ROS 2 integration
2. A basic understanding of the Gazebo interface
3. Created a simple world with objects
4. Verified that physics simulation is working
5. Confirmed ROS 2 communication with Gazebo is functional

## Troubleshooting Tips

- **Gazebo crashes on startup**: Check that you have the required graphics drivers installed
- **Cannot connect to gzserver**: Make sure no other instances of Gazebo are running
- **No ROS topics available**: Ensure ROS 2 is properly sourced in your terminal
- **Slow performance**: Close other applications to free up system resources
- **Rendering issues**: Try running with software rendering: `gazebo --verbose --render-engine=ogre2`

## Comprehension Check

1. What is the primary command to launch Gazebo?
   - A) `gazebo start`
   - B) `gz sim`
   - C) `launch gazebo`
   - D) Both B and the simple `gazebo` command
   
   **Correct Answer:** D
   **Explanation:** Both the `gazebo` command and the newer `gz sim` command can be used to launch Gazebo simulation.

2. What is required to use Gazebo with ROS 2?
   - A) No additional packages needed
   - B) Only ROS 2 installed
   - C) ROS 2 and gazebo_ros_pkgs
   - D) A special ROS 2 distribution
   
   **Correct Answer:** C
   **Explanation:** To integrate Gazebo with ROS 2, you need both ROS 2 installed and the appropriate Gazebo ROS packages (gazebo_ros_pkgs).

3. What happens when you click the "Play" button in Gazebo?
   - A) The simulation is saved
   - B) Physics simulation begins
   - C) A video recording starts
   - D) The world is exported to SDF
   
   **Correct Answer:** B
   **Explanation:** The "Play" button starts the physics simulation, enabling objects to respond to forces and collisions.

## Summary

This lab provided hands-on experience with setting up Gazebo and creating a basic simulation environment. You learned how to launch Gazebo, add objects, configure basic properties, and verify the physics simulation. You also confirmed the integration between Gazebo and ROS 2, which is essential for robotics applications.

## References

1. Open Source Robotics Foundation. (2023). Gazebo Documentation. Gazebo Simulator. http://gazebosim.org/
2. Open Source Robotics Foundation. (2023). ROS 2 with Gazebo. ROS Documentation.
3. Quigley, M., et al. (2009). ROS: an open-source Robot Operating System. ICRA Workshop on Open Source Software.
4. Gerkey, B., et al. (2003). The Player/Stage Project: Tools for Multi-Robot and Distributed Sensor Systems. In Proceedings of the 11th International Conference on Advanced Robotics.
5. Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics. Springer.