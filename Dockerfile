# ============================================================
# BASE: Clean Ubuntu 24.04 with CUDA support (NO drivers)
# ============================================================
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
WORKDIR /workspace

# ------------------------------------------------------------
# LAYER 1: Prerequisites and NVIDIA CUDA repository
# ------------------------------------------------------------
RUN apt-get update && \
    WORKDIR /workspace
    CMD ["bash"]
    software-properties-common git python3-pip wget build-essential && \
    curl -fSsl https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb -o /tmp/cuda-keyring.deb && \
    dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb && \
    apt-get update

# ------------------------------------------------------------
# LAYER 2: Dynamic Linker for WSLg graphics
# ------------------------------------------------------------
RUN echo "/usr/lib/wsl/lib" > /etc/ld.so.conf.d/wsl.conf && ldconfig

# ------------------------------------------------------------
# LAYER 3: ROS 2 Jazzy and Gazebo repositories
# ------------------------------------------------------------
RUN add-apt-repository universe && \
    curl -sSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# ------------------------------------------------------------
# LAYER 4: Install CUDA, ROS 2 Jazzy, Gazebo, and Dependencies
# ------------------------------------------------------------
RUN apt-get install -y --no-install-recommends \
    # --- CUDA Toolkit (no display drivers)
    cuda-toolkit-12-5 \
    # --- Graphics libs (for WSLg/GLX)
    libgl1 libglx-mesa0 libgl1-mesa-dri mesa-utils x11-apps \
    libxcb-xinerama0 \
    # --- Gazebo Harmonic + ROS Jazzy desktop tools
    gz-harmonic ros-jazzy-desktop \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-route \
    ros-jazzy-slam-toolbox ros-jazzy-ros-gz ros-jazzy-xacro \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    python3-colcon-common-extensions python3-rosdep \
    # --- Extra dependencies from Ackermann image
    ros-jazzy-rcl-interfaces \
    ros-jazzy-rclcpp \
    ros-jazzy-builtin-interfaces \
    ros-jazzy-sdformat-urdf \
    ros-jazzy-vision-msgs \
    ros-jazzy-actuator-msgs \
    ros-jazzy-image-transport \
    ros-jazzy-behaviortree-cpp-v3 && \
    rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# LAYER 5: Initialize rosdep and auto-source setup
# ------------------------------------------------------------
RUN rosdep init && rosdep update
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

# ------------------------------------------------------------
# LAYER 6: Clone additional repos and build
# ------------------------------------------------------------
# Optional repo: NVLabs tiny-cuda-nn
ARG REPO_URL="https://github.com/NVlabs/tiny-cuda-nn.git"
RUN git clone --depth 1 ${REPO_URL} && \
    cd tiny-cuda-nn && if [ -f requirements.txt ]; then pip3 install -r requirements.txt; fi

# Clone car_nav2
RUN git clone https://github.com/2024828/car_nav2.git /workspace/car_navigation/src/car_nav2

# Clone interactive marker server
RUN git clone https://github.com/ros-visualization/interactive_marker_twist_server.git /workspace/car_navigation/src/interactive_marker_twist_server

# Clone ackermann sim
RUN mkdir -p /root/colcon_ws/src && \
    git clone https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2.git /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2

# ------------------------------------------------------------
# LAYER 7: Build both workspaces
# ------------------------------------------------------------
# Build car_navigation
WORKDIR /workspace/car_navigation
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build"

# Build ackermann vehicle sim
WORKDIR /root/colcon_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build"

# ------------------------------------------------------------
# LAYER 8: Environment paths and final setup
# ------------------------------------------------------------
ENV GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2
ENV ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2

RUN echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/car_navigation/install/setup.bash" >> /root/.bashrc

WORKDIR /workspace
CMD ["bash"]
