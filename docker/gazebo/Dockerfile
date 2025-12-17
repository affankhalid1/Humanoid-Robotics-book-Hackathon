# Use ROS 2 Humble with Gazebo Harmonic
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Install Gazebo and ROS 2 Gazebo packages
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-dev \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Install additional utilities
RUN apt-get update && apt-get install -y \
    git \
    curl \
    vim \
    nano \
    wget \
    unzip \
    build-essential \
    cmake \
    pkg-config \
    libeigen3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /workspace
SHELL ["/bin/bash", "-c"]

# Source ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create a catkin workspace
RUN mkdir -p /workspace/src

# Copy any custom packages or configurations (if needed)
# COPY . /workspace/src/

# Build the workspace
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select"

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]