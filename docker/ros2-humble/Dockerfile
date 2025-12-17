# Use ROS 2 Humble Hawksbill base image
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Set working directory
WORKDIR /workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    git \
    wget \
    curl \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    matplotlib \
    opencv-python \
    pyyaml \
    flask \
    requests

# Install rosdep dependencies
RUN rosdep update

# Copy ROS 2 workspace setup
RUN mkdir -p /workspace/src

# Setup ROS 2 environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create a non-root user for security
RUN useradd -m -s /bin/bash -u 1000 rosuser && \
    usermod -aG dialout rosuser
USER rosuser
WORKDIR /home/rosuser/workspace

# Source ROS 2 environment for the user
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]