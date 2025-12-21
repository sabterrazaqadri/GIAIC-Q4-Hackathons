# ROS 2 Workspace Setup for Testing Examples

## Overview
This document outlines the setup of a ROS 2 workspace specifically for testing examples in the Physical AI & Humanoid Robotics textbook modules. The workspace will support the examples and lab exercises in Modules 2-4.

## Workspace Structure
```
ros2_workspace/
├── src/                 # Source code for packages
│   ├── robot_description/    # URDF/XACRO models
│   ├── simulation_envs/      # Simulation environment configurations
│   ├── perception_examples/  # Perception pipeline examples
│   ├── navigation_examples/  # Navigation examples
│   └── vla_examples/         # Vision-Language-Action examples
├── build/               # Build space
├── install/             # Install space
└── log/                 # Log files
```

## Installation and Setup

### 1. Install ROS 2 Humble Hawksbill
```bash
# On Ubuntu 22.04
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool
sudo apt install python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Create the Workspace
```bash
# Create workspace directory
mkdir -p ~/ros2_textbook_ws/src
cd ~/ros2_textbook_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace (even though it's empty)
colcon build --packages-select
```

## Core Packages for Textbook Examples

### 1. Robot Description Package
```bash
# Create robot_description package
cd ~/ros2_textbook_ws/src
ros2 pkg create --build-type ament_cmake robot_description --dependencies urdf xacro

# robot_description/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY
  launch
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# robot_description/package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_description</name>
  <version>0.0.1</version>
  <description>Robot description for textbook examples</description>
  <maintainer email="textbook@physicalai.com">Physical AI Textbook Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>urdf</depend>
  <depend>xacro</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 2. Simulation Environments Package
```bash
# Create simulation_envs package
cd ~/ros2_textbook_ws/src
ros2 pkg create --build-type ament_python simulation_envs

# simulation_envs/setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'simulation_envs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Physical AI Textbook Team',
    maintainer_email='textbook@physicalai.com',
    description='Simulation environments for textbook examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

### 3. Perception Examples Package
```bash
# perception_examples/package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>perception_examples</name>
  <version>0.0.1</version>
  <description>Perception examples for textbook</description>
  <maintainer email="textbook@physicalai.com">Physical AI Textbook Team</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>vision_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

# perception_examples/setup.py
import os
from glob import glob
from setuptools import setup

package_name = 'perception_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Physical AI Textbook Team',
    maintainer_email='textbook@physicalai.com',
    description='Perception examples for textbook modules',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_perception = perception_examples.simple_perception:main',
            'object_detection = perception_examples.object_detection:main',
        ],
    },
)
```

## Example Launch Files

### Isaac Sim Bridge Launch
```python
# simulation_envs/launch/isaac_sim_bridge.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='textbook_humanoid',
        description='Robot model to load'
    )
    
    # Isaac Sim ROS bridge node
    isaac_sim_bridge = Node(
        package='isaac_ros_common',
        executable='isaac_ros_bridge',
        name='isaac_sim_bridge',
        parameters=[
            {'robot_model': LaunchConfiguration('robot_model')},
            {'use_sim_time': True}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    
    # RViz node for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', ['/usr/share/rviz2/launch/default.rviz']],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        robot_model,
        isaac_sim_bridge,
        rviz
    ])
```

### Gazebo Launch with Robot
```python
# simulation_envs/launch/gazebo_with_robot.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Gazebo with a world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('simulation_envs'),
                'worlds',
                'living_room.world'
            ])
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'textbook_robot',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.5'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_entity
    ])
```

## Basic Perception Node Example
```python
# perception_examples/perception_examples/simple_perception.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SimplePerceptionNode(Node):
    def __init__(self):
        super().__init__('simple_perception')
        
        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Create publisher for processed image
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_processed',
            10
        )
        
        self.bridge = CvBridge()
        
    def scan_callback(self, msg):
        # Process laser scan data
        # Example: detect obstacles within 1 meter
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            if min_distance < 1.0:
                self.get_logger().info(f'Obstacle detected at {min_distance:.2f} meters')
    
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Example: Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Convert back to ROS Image message
        processed_msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
        processed_msg.header = msg.header
        
        # Publish processed image
        self.image_pub.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    
    perception_node = SimplePerceptionNode()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Workspace Setup Script
```bash
#!/bin/bash
# setup_workspace.sh - Script to set up the ROS 2 workspace for textbook examples

echo "Setting up ROS 2 workspace for Physical AI & Humanoid Robotics textbook..."

# Create workspace if it doesn't exist
if [ ! -d "$HOME/ros2_textbook_ws" ]; then
    mkdir -p $HOME/ros2_textbook_ws/src
    echo "Created workspace directory: $HOME/ros2_textbook_ws"
fi

# Navigate to workspace
cd $HOME/ros2_textbook_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Create packages if they don't exist
if [ ! -d "src/robot_description" ]; then
    cd src
    ros2 pkg create --build-type ament_cmake robot_description --dependencies urdf xacro
    cd ..
fi

if [ ! -d "src/simulation_envs" ]; then
    cd src
    ros2 pkg create --build-type ament_python simulation_envs
    cd ..
fi

if [ ! -d "src/perception_examples" ]; then
    cd src
    ros2 pkg create --build-type ament_python perception_examples --dependencies rclpy sensor_msgs std_msgs cv_bridge vision_msgs
    cd ..
fi

# Build the workspace
colcon build --packages-select robot_description simulation_envs perception_examples

# Source the workspace
source install/setup.bash

echo "ROS 2 workspace setup complete!"
echo "To use the workspace in new terminals, run: source $HOME/ros2_textbook_ws/install/setup.bash"
echo "Or add the following line to your ~/.bashrc file:"
echo "source $HOME/ros2_textbook_ws/install/setup.bash"
```

## Example Usage for Textbook Labs

### Lab 1: Basic ROS 2 Communication
```python
# Example publisher and subscriber for Module 2
# In a file: ~/ros2_textbook_ws/src/basic_communication/basic_communication/talker.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, robot! Message #{self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This ROS 2 workspace setup provides the foundation for all the examples and lab exercises in the Physical AI & Humanoid Robotics textbook. Students will be able to build, run, and experiment with the code examples provided in each module.