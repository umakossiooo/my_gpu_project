# 1. Start with a clean Ubuntu 24.04 base image. This avoids any pre-installed driver conflicts.
FROM ubuntu:24.04

# Set non-interactive frontend to avoid prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Set the working directory inside the container
WORKDIR /workspace

# === LAYER 1: Install prerequisites and add the official NVIDIA CUDA repository ===
# We will install only the CUDA toolkit, NOT the drivers.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    apt-utils \
    debconf-utils \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    git \
    python3-pip && \
    # Add the NVIDIA CUDA repository for Ubuntu 24.04
    curl -fSsl https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb -o /tmp/cuda-keyring.deb && \
    dpkg -i /tmp/cuda-keyring.deb && \
    rm /tmp/cuda-keyring.deb && \
    apt-get update

# === LAYER 2: Permanently Configure the Dynamic Linker for WSLg ===
# This is the definitive fix. We are telling the core system linker to always
# search the WSL graphics library path. This cannot be overridden by shell scripts.
RUN echo "/usr/lib/wsl/lib" > /etc/ld.so.conf.d/wsl.conf && ldconfig

# === LAYER 3: Add ROS, Gazebo repositories ===
RUN add-apt-repository universe && \
    curl -sSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# === LAYER 4: Install CUDA Toolkit, ROS, Gazebo and all other packages ===
RUN apt-get install -y --no-install-recommends \
    # Install the CUDA toolkit (for compute) WITHOUT the conflicting display drivers
    cuda-toolkit-12-5 \
    # --- Graphics libraries (Mesa is needed for GLX, but will use NVIDIA via WSLg) ---
    libgl1 \
    libglx-mesa0 \
    libgl1-mesa-dri \
    mesa-utils \
    x11-apps \
    # ***** FIX ADDED HERE *****
    # This is the critical package that Qt's XCB plugin needs to connect to the display.
    libxcb-xinerama0 \
    # --- ROS & Gazebo Packages ---
    gz-harmonic \
    ros-jazzy-desktop \
    python3-colcon-common-extensions \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-route \
    ros-jazzy-slam-toolbox \
    python3-rosdep \
    ros-jazzy-ros-gz \
    ros-jazzy-xacro \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers && \
    # Clean up apt lists to reduce final image size
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep system-wide
RUN rosdep init && rosdep update

# Source the ROS 2 setup script automatically
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

# --- Your Project's Non-ROS Dependencies ---
ARG REPO_URL="https://github.com/NVlabs/tiny-cuda-nn.git"
RUN git clone --depth 1 ${REPO_URL}
WORKDIR /workspace/tiny-cuda-nn
RUN if [ -f requirements.txt ]; then pip3 install -r requirements.txt; fi

# Clone car_nav2 into the src directory
RUN git clone https://github.com/2024828/car_nav2.git /workspace/car_navigation/src/car_nav2

# === Build the ROS Workspace ===
WORKDIR /workspace
RUN mkdir -p /workspace/car_navigation/src
RUN git clone https://github.com/ros-visualization/interactive_marker_twist_server.git /workspace/car_navigation/src/interactive_marker_twist_server
WORKDIR /workspace/car_navigation
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build"

# Source the new workspace setup file automatically
RUN echo 'source /workspace/car_navigation/install/setup.bash' >> /root/.bashrc

# Set the final working directory
WORKDIR /workspace

# Start a bash shell
CMD ["bash"]
