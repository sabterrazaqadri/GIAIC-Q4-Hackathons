# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Overview

Welcome to the Physical AI & Humanoid Robotics textbook! This guide will help you set up your environment to follow along with Modules 2-4, covering simulation, AI perception, and Vision-Language-Action (VLA) pipelines.

## Prerequisites

Before starting, ensure you have:
- A computer with: 8+ GB RAM, dedicated GPU recommended (for simulation)
- Operating System: Ubuntu 22.04 LTS, Windows 10+, or macOS 12+
- Python 3.10+
- Git installed
- Docker (recommended for isolated environments)
- Basic knowledge of robotics and programming concepts

## Setting Up the Documentation Environment

### 1. Clone the Repository

```bash
git clone https://github.com/[your-org]/physical-ai-humanoid-textbook.git
cd physical-ai-humanoid-textbook
```

### 2. Install Docusaurus Dependencies

```bash
cd docusaurus
npm install
```

### 3. Start the Local Documentation Server

```bash
npm start
```

This will launch the textbook website at `http://localhost:3000`.

## Setting Up the Simulation Environment

### Option A: Isaac Sim Setup (Recommended for Module 2)

1. Download Isaac Sim from NVIDIA Omniverse (requires registration)
2. Install Isaac Sim following the official installation guide
3. Install Isaac Sim Python API:
   ```bash
   # Follow Isaac Sim installation instructions for Python API setup
   ```
4. Set up the robot models:
   ```bash
   # Refer to Module 2, Chapter 1 for specific robot model setup instructions
   ```
5. Test the installation:
   ```bash
   # Run a basic simulation as outlined in the textbook's first lab
   ```

### Option B: Gazebo Setup

1. Install ROS 2 Humble Hawksbill:
   ```bash
   # Follow ROS 2 installation instructions for your OS
   ```
2. Install Gazebo Garden:
   ```bash
   # Follow Gazebo installation instructions
   ```
3. Install additional packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-gazebo-* ros-humble-navigation2 ros-humble-nav2-bringup
   ```
4. Test the installation:
   ```bash
   # Launch a basic simulation as outlined in the textbook
   ```

## Setting Up AI and Perception Pipelines

### 1. Install Required Python Packages

```bash
pip install numpy opencv-python torch torchvision torchaudio
pip install openai-whisper  # For speech-to-text
pip install openai  # For LLM integration
pip install rclpy  # For ROS 2 Python client library
pip install sensor_msgs geometry_msgs nav_msgs  # ROS 2 message types
```

### 2. Configure AI Services

1. Obtain an OpenAI API key from [https://platform.openai.com/api-keys](https://platform.openai.com/api-keys)
2. Set up your environment variables:
   ```bash
   export OPENAI_API_KEY='your-api-key-here'
   ```

## Starting with the Modules

### Module 2: The Digital Twin (Simulation Environment)

Begin with Chapter 1, which will guide you through:
- Setting up your first simulation environment
- Understanding physics simulation concepts
- Working with sensor simulation
- Running your first simulation scenario

### Module 3: The AI-Robot Brain (Perception & Navigation)

Once comfortable with simulation, proceed to:
- Setting up perception pipelines
- Understanding visual SLAM and navigation
- Working with synthetic data generation
- Exploring sim-to-real transfer concepts

### Module 4: Vision-Language-Action (VLA)

Finally, explore the intersection of AI and robotics:
- Implementing speech-to-text conversion
- Creating LLM-driven task planning
- Converting natural language to robotic actions
- Integrating safety and validation layers

## Lab Exercise Workflow

Each chapter includes lab exercises with the following typical workflow:

1. **Read** the lab objectives and background information
2. **Set up** the required environment and files
3. **Execute** the step-by-step commands
4. **Validate** the results against expected outputs
5. **Troubleshoot** using provided tips if issues arise

Example lab structure:
```
lab-exercise/
├── setup.sh                # Environment setup script
├── run.py                  # Main execution script
├── expected_output.txt     # Expected results
├── troubleshooting.md      # Common issues and solutions
└── config.yaml             # Configuration parameters
```

## Troubleshooting Common Issues

### Simulation Performance
- If experiencing performance issues, adjust graphics settings or reduce simulation complexity
- For Isaac Sim, try using lower-quality rendering modes during development

### Python Package Conflicts
- Use virtual environments to isolate dependencies
- Some packages may require specific Python or CUDA versions

### ROS 2 Communication Issues
- Ensure ROS 2 environment is sourced in each terminal: `source /opt/ros/humble/setup.bash`
- Check that ROS_DOMAIN_ID is consistent across terminals

## Getting Support

- Report issues with textbook content in the GitHub repository
- Join the discussion forum for questions about concepts
- Check the troubleshooting section in each chapter for module-specific issues

## Next Steps

After completing this quickstart:
- Proceed to Module 2, Chapter 1
- Familiarize yourself with the simulation environment
- Complete the first lab exercise to validate your setup
- Begin exploring the academic citations referenced in each chapter to deepen your understanding