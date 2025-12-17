# Use ROS 2 Humble with Python support
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install Python and pip
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install audio processing dependencies
RUN apt-get update && apt-get install -y \
    portaudio19-dev \
    ffmpeg \
    sox \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch with CUDA support (if available)
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install Whisper and related packages
RUN pip3 install openai-whisper

# Install LangChain and related packages
RUN pip3 install \
    langchain \
    langchain-openai \
    langchain-community \
    langchain-core \
    python-dotenv

# Install additional Python packages
RUN pip3 install \
    pyaudio \
    numpy \
    scipy \
    soundfile \
    webrtcvad \
    pyannote.audio

# Install ROS 2 Python packages
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    rclpy \
    std-msgs \
    sensor-msgs

# Set up the workspace
WORKDIR /workspace
SHELL ["/bin/bash", "-c"]

# Source ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create a catkin workspace
RUN mkdir -p /workspace/src

# Copy VLA interfaces (if any)
# COPY vla_interfaces /workspace/src/vla_interfaces

# Build the workspace (if needed)
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select vla_interfaces"

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]