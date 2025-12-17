# Use Isaac Sim base image
FROM nvcr.io/nvidia/isaac-sim:4.0.0

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV ISAAC_SIM_PATH=/isaac-sim
ENV PYTHONPATH=/isaac-sim/python
ENV NVIDIA_GL_LOG=1

# Install additional dependencies for Isaac ROS integration
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    pkg-config \
    git \
    curl \
    vim \
    nano \
    wget \
    unzip \
    libeigen3-dev \
    libopencv-dev \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-isaac-ros-dev \
    ros-humble-isaac-ros-common \
    ros-humble-isaac-ros-perception \
    ros-humble-isaac-ros-nav2 \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for Isaac ROS
RUN pip3 install --upgrade pip && \
    pip3 install \
    opencv-python \
    numpy \
    scipy \
    matplotlib \
    torch \
    torchvision \
    tensorrt \
    pycuda

# Set up the workspace
WORKDIR /workspace
SHELL ["/bin/bash", "-c"]

# Source ROS 2 and Isaac Sim
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /isaac-sim/isaac-sim.sh" >> ~/.bashrc

# Create a catkin workspace
RUN mkdir -p /workspace/src

# Set Isaac Sim permissions
RUN chmod -R 755 /isaac-sim

# Expose ports for Isaac Sim
EXPOSE 55555-55559/udp
EXPOSE 8211/udp

# Set the entrypoint
ENTRYPOINT ["/isaac-sim/isaac-sim.sh"]
CMD ["bash"]