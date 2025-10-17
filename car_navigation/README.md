# Quick Start Guide: Car Navigation Simulation (ROS 2 Jazzy + Gazebo Harmonic)

## Prerequisites
- Docker and Docker Compose installed
- NVIDIA GPU and drivers (for GPU acceleration)
- NVIDIA Container Toolkit installed on host
- X11 server running (for GUI)

## 1. Build the Docker Image
From your project root:
```bash
docker-compose build
# Build the Docker image using cache (faster if you haven't changed Dockerfile or dependencies)

docker-compose build --no-cache
# Build the Docker image without using any cache (forces a full rebuild, useful if you want to ensure all layers are rebuilt)

docker compose pull
# Pull the latest images for services from remote repositories (if defined in docker-compose.yml)

docker compose build && docker compose up
```

## 2. Enable X11 Forwarding (for GUI)
On your host (Linux):
```bash
xhost +local:root
```

## 3. Start the Container
```bash
docker-compose up
# Start the container and attach to its logs/output in the foreground

docker compose up -d
# Start the container in detached mode (runs in the background)

docker compose up --build -d
# Build the image first, then start the container in detached mode (useful if you want to ensure the latest build is used)
```
All these commands will start the container(s) as defined in your docker-compose.yml. Use the detached mode if you don't want the terminal to be blocked by container logs.

## 4. Enter the Container (if not already attached)
```bash
docker-compose exec simulation bash
```

## 5. Set Up Gazebo Models Path
Inside the container:
```bash
export GZ_SIM_RESOURCE_PATH=/workspace/car_navigation/gazebo_models
```

## 6. Build the ROS 2 Workspace
```bash
cd /workspace/car_navigation
colcon build
```

## 7. Source the ROS 2 Setup Script
```bash
source install/setup.bash
```

## 8. Launch the Simulation
```bash
ros2 launch car_nav2 spawn_robot.launch.py
```

## 9. (Optional) Run Navigation in Another Terminal
Open a new terminal, enter the container, and run:
```bash
export GZ_SIM_RESOURCE_PATH=/workspace/car_navigation/gazebo_models
source install/setup.bash
ros2 launch car_nav2 navigation_with_slam.launch.py
```

## 10. Using RViz2
- RViz2 should open automatically if configured in your launch file.
- Use the `2D goal pose` tool to send navigation goals.
- For waypoint navigation, add the `NAV2 Goal` tool and use the Nav2 plugin.

## Troubleshooting
- If Gazebo GUI fails, check X11 forwarding and Docker Compose settings:
  - Ensure these lines are in your `docker-compose.yml`:
    ```yaml
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    ```
- If you are on Windows and using an X server (like VcXsrv or Xming):
  - Start the X server with **`Disable access control`** and **`Native OpenGL`** enabled.
  - Make sure the X server is running before starting your container.
  - Set `export DISPLAY=:0` in your WSL2 or container shell.
- If GUI applications fail to open or you see display errors:
  - Inside your container, set the DISPLAY variable:
    ```bash
    export DISPLAY=:0
    ```
  - Test X11 forwarding with:
    ```bash
    xeyes
    glxgears
    ```
  - If these open windows on your host, X11 forwarding is working.
- For headless simulation, use:
  ```bash
  gz sim --headless /workspace/car_navigation/src/worlds/home.sdf
  ```
- Always run `source install/setup.bash` in every new shell before launching ROS 2 nodes.

---
For more details, see the main README below.

# car_nav2
Using ROS2 Jazzy and Gazebo Harmonic for autonomous navigation simulation of a robot car.

This project is based on the BME MOGI - ROS course, with the camera part removed, and the mapping and navigation features retained. For further learning, please refer to https://github.com/MOGI-ROS.

This project is for personal learning purposes only. If there are any copyright infringements, please contact us for removal.

You need to do this:

1.Run the following command to clone the car_nav2 repository to your local machine:

    git clone https://github.com/2024828/car_nav2.git

2.The file uses some Gazebo models, which you can download from:https://drive.google.com/file/d/1tcfoLFReEW1XNHPUAeLpIz2iZXqQBvo_/view.

If you want to learn more about Gazebo models, please visit:https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics.

Make sure to let Gazebo know about their location by running:

    export GZ_SIM_RESOURCE_PATH=/workspace/car_navigation/gazebo_models

    * fix this depending on the path of gazebo_models

3.Build the package and run:
    
    colcon build
    . install/setup.bash
    ros2 launch car_nav2 spawn_robot.launch.py

Open a new terminal and run:
    
    export GZ_SIM_RESOURCE_PATH=~/gazebo_models
    . install/setup.bash
    ros2 launch car_nav2 navigation_with_slam.launch.py

You can use the `2D goal pose` in the RViz2 toolbar to control the robot's movement and mapping. The robot will automatically plan the path.

You can also click the `+` on rviz2 to add the `NAV2 Goal` tool. Then, click on the `Waypoint/NavThrough Poses Mode` under the Nav2 plugin. You can use the `Nav2 Goal` to set multiple waypoints in sequence. By clicking the `StartWaypoint Following` at the bottom, you can start waypoint navigation, and the robot will move towards the preset target locations in order.
