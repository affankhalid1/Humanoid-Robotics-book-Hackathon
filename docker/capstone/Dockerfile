# Use Isaac Sim as base for full integration
FROM nvcr.io/nvidia/isaac-sim:4.0.0

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV ISAAC_SIM_PATH=/isaac-sim
ENV PYTHONPATH=/isaac-sim/python

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe \
    && apt-get update \
    && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
    ros-humble-ros-base \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install additional dependencies for all modules
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    unzip \
    portaudio19-dev \
    ffmpeg \
    sox \
    libeigen3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for all modules
RUN pip3 install --upgrade pip && \
    pip3 install \
    rclpy \
    std-msgs \
    geometry-msgs \
    sensor-msgs \
    nav-msgs \
    torch \
    torchvision \
    torchaudio \
    openai-whisper \
    langchain \
    langchain-openai \
    langchain-community \
    langchain-core \
    python-dotenv \
    pyaudio \
    numpy \
    scipy \
    matplotlib \
    opencv-python \
    webrtcvad \
    pyannote.audio \
    tensorrt \
    pycuda

# Install Isaac ROS packages
RUN apt-get update && apt-get install -y \
    ros-humble-isaac-ros-dev \
    ros-humble-isaac-ros-common \
    ros-humble-isaac-ros-perception \
    ros-humble-isaac-ros-nav2 \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /workspace
SHELL ["/bin/bash", "-c"]

# Source ROS 2 and Isaac Sim
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /isaac-sim/isaac-sim.sh" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Create a catkin workspace
RUN mkdir -p /workspace/src

# Set Isaac Sim permissions
RUN chmod -R 755 /isaac-sim

# Expose ports for Isaac Sim and ROS 2
EXPOSE 55555-55559/udp
EXPOSE 8211/udp
EXPOSE 11311  # ROS 2 master

# Set the entrypoint
ENTRYPOINT ["/isaac-sim/isaac-sim.sh"]
CMD ["bash"]