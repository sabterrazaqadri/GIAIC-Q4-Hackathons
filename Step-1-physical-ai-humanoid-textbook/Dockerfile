# Dockerfile for ROS 2 Humble Development Environment
# For use with the Physical-AI & Humanoid-Robotics textbook Module 1

# Use ROS 2 Humble Hawksbill base image
FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Set working directory
WORKDIR /workspace

# Install basic utilities and development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    curl \
    vim \
    wget \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages needed for Module 1
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true

# Clone the textbook repository
RUN git clone https://github.com/your-username/your-repository.git /workspace/textbook

# Create a workspace for the examples
RUN mkdir -p /workspace/ros2_workspace/src

# Copy examples to the workspace
COPY examples/ /workspace/ros2_workspace/src/

# Install Python dependencies
RUN pip3 install --upgrade pip && \
    pip3 install numpy matplotlib

# Source ROS 2 environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Set up the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /workspace/ros2_workspace && \
    colcon build

# Update bashrc to source ROS environment and workspace
RUN echo "source /workspace/ros2_workspace/install/setup.bash" >> ~/.bashrc

# Create directory for assets
RUN mkdir -p /workspace/textbook/assets

# Copy assets
COPY assets/ /workspace/textbook/assets/

# Set default command to start bash
CMD ["bash"]