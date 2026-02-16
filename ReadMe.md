# Hydrogen and Deuterium – Team AUV MIT-B

Building our 2nd and 3rd prototype to participate in RoboSub 2026.


## Overview
We've created Hydrogen and Deuterium to replicate the complete RoboSub 2025 competition track for fully autonomous navigation. The modular file structure makes multi-person collaboration seamless across our team.

We use it for preliminary testing of all tech stacks—from Electrical Team stabilization controllers to manual teleop interfaces. We're currently implementing initial autonomy through SAUVC tasks using Behaviour Trees, progressing toward full track navigation with our custom-trained object detection pipeline. This sim-based rapid prototyping gives us iteration flexibility before hardware deployment.

This project utilizes:
- **ROS 2** for communication
- **Gazebo** for simulation

## Prerequisites

- **ROS 2**: Humble
- **Gazebo**: Ignition Fortress

Ensure that ROS 2 Humble and Gazebo Ignition Fortress are correctly installed and sourced on your system.

## Installation

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/MITB-AUVTeam/hydrogen.git
    cd hydrogen
    ```

2.  **Build the workspace**:
    ```bash
    colcon build
    ```

3.  **Source the setup file**:
    ```bash
    source install/setup.bash
    ```

4. **Launch the model file**:
    ```bash 
    ros2 launch hydrogen model.launch.py
    ```

## Project Structure

- **launch/**: Contains Python launch files for the hydrogen/deuterium models.
- **model/**: Robot description files, which have been made modular based on each aspect of the bot from the main_frame to propellers to Zed cameras etc.
- **world/**: Gazebo world files (SDF), a completely underwater environment with relevant physics and hydrodynamics simulating the complete competition track from RoboSub 2025.
- **parameters/**: Configuration files.
- **meshes/**: 3D models for robots, primarily STL files designed in CAD by Mechanical Team members.

# NOTE :-

Before statring the simulation you need to set the enviorinment variable for your gazebo plugin directory, which includes a custom plugin written by one of our team members.


```
echo 'export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:<path to your plugin folder>' >> ~/.bashrc
```
use pwd command to find the path to your plugins folder.
