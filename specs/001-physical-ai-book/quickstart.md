# Quickstart Guide: Physical AI Humanoid Robotics Book

## Prerequisites

### Hardware Requirements
- **Minimum**: 8+ core CPU, 16GB+ RAM, NVIDIA GPU with 4GB+ VRAM
- **Recommended**: 12+ core CPU, 32GB RAM, NVIDIA RTX 4070 Ti or better with 12GB+ VRAM
- **Storage**: 50GB+ free space for ROS 2, Isaac Sim, and Docker images

### Software Requirements
- Ubuntu 22.04 LTS (primary development environment)
- Windows/Mac users: WSL2 with Ubuntu 22.04 or Virtual Machine
- Docker and Docker Compose
- Git version control
- Python 3.10+
- Node.js 18+ and npm


## Setup Process

### 1. Clone the Repository
```bash
git clone https://github.com/your-username/physical-ai-humanoid-robotics.git
cd physical-ai-humanoid-robotics
```

### 2. Install System Dependencies
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble dependencies
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop ros-humble-cv-bridge ros-humble-tf2-tools python3-colcon-common-extensions python3-rosdep python3-vcstool
source /opt/ros/humble/setup.bash

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sh get-docker.sh
sudo usermod -aG docker $USER
newgrp docker
```

### 3. Set Up Development Environment
```bash
# Install Python dependencies
pip3 install --user rosdep vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Install Node.js and npm (if not already installed)
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs
```

### 4. Build Docker Images
```bash
# Navigate to docker directory
cd docker

# Build ROS 2 Humble image
cd ros2-humble
docker build -t ros2-humble-book:latest .

# For Isaac Sim (requires NVIDIA GPU and drivers)
cd ../isaac-sim
docker build -t isaac-sim-book:latest .

# Return to root
cd ..
```

### 5. Run the Development Environment
```bash
# For ROS 2 development
cd docker/ros2-humble
docker-compose up -d

# Access the container
docker exec -it ros2-book-container bash

# For Isaac Sim (requires NVIDIA GPU)
cd docker/isaac-sim
# Note: Isaac Sim setup requires NVIDIA Omniverse account and specific setup
# Follow the Isaac Sim documentation for complete setup
```

## Running Code Examples

### 1. Navigate to a Module
```bash
cd module-01-ros2/chapter-01-core-concepts/examples
```

### 2. Build and Run
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source the built packages
source install/setup.bash

# Run a specific example
ros2 run your_package_name your_node_name
```

### 3. Using Docker for Examples
```bash
# Run in isolated environment
docker run -it --rm -v $(pwd):/workspace ros2-humble-book:latest bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Viewing the Book

### 1. Install Docusaurus Dependencies
```bash
cd physical-ai-humanoid-robotics
npm install
```

### 2. Start Local Development Server
```bash
npm start
```

### 3. Access the Book
Open your browser to `http://localhost:3000` to view the book locally.

## Testing Code Examples

### 1. Run Automated Tests
```bash
# From repository root
bash scripts/test-runner.sh

# Or run specific module tests
cd module-01-ros2
bash test-runner.sh
```

### 2. Validate Code References
```bash
python scripts/validate_code_references.py
```

### 3. Run Pre-deployment Checks
```bash
bash scripts/pre-deploy-check.sh
```

## Troubleshooting Common Issues

### ROS 2 Issues
- **Package not found**: Ensure you've sourced the correct setup.bash file
- **Node not launching**: Check for missing dependencies with `rosdep check`
- **Permission errors**: Ensure Docker is running with proper permissions

### Isaac Sim Issues
- **GPU not detected**: Verify NVIDIA drivers are installed and CUDA is working
- **Performance issues**: Ensure RTX GPU with sufficient VRAM is available
- **License issues**: Follow Isaac Sim documentation for proper licensing

### Docker Issues
- **Permission denied**: Add user to docker group and log out/in
- **Port conflicts**: Check for other services using the same ports
- **Image build failures**: Ensure sufficient disk space and internet connection

## Next Steps

1. **Introduction Module**: Start with `docs/intro/welcome.md` to understand Physical AI concepts
2. **ROS 2 Module**: Move to `docs/module-01-ros2/` for the foundational ROS 2 concepts
3. **Code Examples**: Follow along with examples in `module-01-ros2/chapter-01-core-concepts/examples/`
4. **Projects**: Complete the hands-on projects to reinforce learning

## Getting Help

- Check the troubleshooting section in each module
- Review the appendices for reference materials
- Report issues on the GitHub repository
- Join the ROS community forums for additional support