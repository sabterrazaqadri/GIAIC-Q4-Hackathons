# Physical-AI & Humanoid-Robotics Textbook - Module 1

This repository contains the Physical-AI & Humanoid-Robotics textbook, with the first module focusing on ROS 2 fundamentals for humanoid robotics.

## Module 1: ROS 2 Fundamentals for Humanoid Robotics

Module 1 covers the fundamentals of ROS 2 as the "robotic nervous system", including:

- ROS 2 Nodes, Topics, Services, and real-time communication
- Connecting OpenAI Agents / Python agents to ROS 2 controllers using rclpy
- Understanding and authoring URDF (Unified Robot Description Format) files for humanoid robots

## Prerequisites

- Ubuntu 20.04 or 22.04
- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Basic Python programming knowledge

## Quick Start with Docker (Recommended)

A Dockerfile is provided for a consistent lab environment:

```bash
# Build the Docker image
docker build -t ros2-humble-textbook .

# Run the container
docker run -it ros2-humble-textbook

# The examples are available in the workspace
cd /workspace/ros2_workspace
source install/setup.bash
```

## Manual Installation

1. Install ROS 2 Humble Hawksbill following the official instructions: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

2. Clone this repository:
```bash
git clone <repository-url>
cd <repository-name>
```

3. Set up your ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

4. Build the example workspace:
```bash
cd /path/to/this/repo
mkdir -p ros2_workspace/src
# Copy the examples to the workspace
cp examples/* ros2_workspace/src/
cd ros2_workspace
colcon build
source install/setup.bash
```

## Running Examples

### Chapter 1: ROS 2 Communication Fundamentals

Run the publisher example:
```bash
python3 examples/publisher-example.py
```

Run the subscriber example:
```bash
python3 examples/subscriber-example.py
```

### Chapter 2: Connecting AI Agents to ROS 2 Controllers

Run the complete AI agent:
```bash
python3 examples/ai-agent-ros-bridge.py
```

### Chapter 3: URDF Fundamentals for Humanoid Robotics

Validate URDF files:
```bash
check_urdf examples/humanoid-limb.urdf
```

Visualize URDF structure:
```bash
ros2 run urdf_parser urdf_to_graphiz examples/humanoid-limb.urdf
```

### Chapter 4: AI-Robot Bridge Integration

Run the full AI humanoid controller:
```bash
python3 examples/ai-humanoid-controller.py
```

## Docusaurus Documentation

The course content is available in Docusaurus format in the `docusaurus/docs/module-1/` directory.

To run the documentation site:
```bash
cd docusaurus
npm install
npm start
```

## Lab Exercises

Complete lab exercises are available in the `module-1/` directory:
- lab-exercise-1-nodes-topics-services.md
- lab-exercise-2-ai-agent-connection.md
- lab-exercise-3-urdf-humanoid-limb.md
- lab-exercise-4-ai-humanoid-task.md

## Structure

```
├── module-1/                 # Module 1 content and lab exercises
├── examples/                 # Python examples for each chapter
├── assets/                   # Diagrams and images
├── docusaurus/              # Docusaurus documentation site
├── tests/                   # Validation scripts
└── specs/                   # Specification files
```

## Contributing

Please read our contributing guidelines before submitting pull requests.

## License

This textbook is licensed under [specify license].