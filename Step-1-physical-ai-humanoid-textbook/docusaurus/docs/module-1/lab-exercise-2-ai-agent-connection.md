# Lab Exercise 2: AI Agent Connection

## Objective

In this lab, you will implement and run a simple AI agent that connects to ROS 2 controllers. You will learn how to create an AI agent that both publishes commands to robot controllers and subscribes to robot sensor data.

## Prerequisites

- Completed Chapter 1 and Chapter 2
- ROS 2 Humble Hawksbill installed
- Basic Python programming knowledge
- Understanding of AI concepts (helpful but not required)

## Estimated Duration

60 minutes

## Instructions

### Part A: Running the Complete AI Agent Example

### Step 1: Set up your workspace
```bash
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
```

### Step 2: Run the complete AI agent node
Open a terminal, source the ROS 2 environment, and run the complete AI agent:
```bash
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
python3 examples/ai-agent-ros-bridge.py
```

### Step 3: Simulate sensor data
In another terminal, simulate LIDAR data that the AI agent will process:
```bash
# Publish a simple LaserScan message
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
ros2 topic pub /scan sensor_msgs/msg/LaserScan "{header: {frame_id: 'laser_frame'}, angle_min: -1.57, angle_max: 1.57, angle_increment: 0.01, time_increment: 0.0, scan_time: 0.0, range_min: 0.0, range_max: 10.0, ranges: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
```

### Step 4: Monitor the AI agent's status
In another terminal, monitor the status published by the AI agent:
```bash
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
ros2 topic echo /ai_agent_status std_msgs/msg/String
```

### Part B: Examining Specific Components

### Step 5: Run the AI publisher example
In a new terminal, run the publisher example:
```bash
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
python3 examples/ai-publisher-example.py
```

### Step 6: Run the AI subscriber example
In another terminal, run the subscriber example with a mock publisher:
```bash
# Terminal 1: Run the subscriber
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
python3 examples/ai-subscriber-example.py

# Terminal 2: Publish mock sensor data
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
ros2 topic pub /scan sensor_msgs/msg/LaserScan "{header: {frame_id: 'laser_frame'}, angle_min: -1.57, angle_max: 1.57, angle_increment: 0.01, time_increment: 0.0, scan_time: 0.0, range_min: 0.0, range_max: 10.0, ranges: [2.0, 2.0, 1.0, 1.0, 0.5, 1.0, 1.0, 2.0, 2.0, 2.0]}"
```

## Expected Output

### For Complete AI Agent:
- The AI agent should publish status messages to `/ai_agent_status`
- The AI agent should publish velocity commands to `/cmd_vel`
- The AI agent should transition between different states (SEARCHING, APPROACHING, ANALYZING)

### For Publisher Example:
- The AI agent should publish status messages and velocity commands periodically
- You should observe the AI agent changing its decision state

### For Subscriber Example:
- The AI agent should receive sensor data and respond by publishing appropriate commands
- When obstacles are detected in the simulated scan, the agent should adjust its movement

## Validation Commands

```bash
# Check active nodes
ros2 node list

# Monitor AI agent status
ros2 topic echo /ai_agent_status std_msgs/msg/String

# Monitor commands sent to robot
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist

# Verify sensor data is being received
ros2 topic echo /scan sensor_msgs/msg/LaserScan

# Check the structure of the AI agent node
ros2 node info /complete_ai_agent
```

## Troubleshooting Tips

- If the navigation service is not available, the AI agent will wait for it
- Make sure the topic names match between publishers and subscribers
- If you get import errors, ensure all required ROS 2 packages are installed
- Use `ros2 doctor` to diagnose common issues

## Extension Activities (Optional)

1. Modify the AI agent to implement a different behavior (e.g., wall following)
2. Add more sensor inputs to the AI agent (e.g., camera or IMU data)
3. Implement a simple machine learning model within the AI agent

## Assessment

Complete the following to verify your understanding:
1. Describe the difference between the AI publisher and subscriber patterns
2. Identify the main components of the complete AI agent
3. Explain how the AI agent processes sensor data to make decisions