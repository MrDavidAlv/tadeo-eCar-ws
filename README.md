# Tadeo eCar 4WD4WS Autonomous Robot

<div align="center">
<img src="images/portada.png" width="280"/>

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](#)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](#)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-orange)](#)
[![Nav2](https://img.shields.io/badge/Nav2-Humble-00599C)](#)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-green)](#)
[![Python](https://img.shields.io/badge/Python-3.8+-yellow?logo=python)](#)
[![Colcon](https://img.shields.io/badge/Build-Colcon-22314E)](#)
[![Git](https://img.shields.io/badge/Git-2.34+-F05032?logo=git)](#)
[![GitHub](https://img.shields.io/badge/GitHub-MrDavidAlv-181717?logo=github)](https://github.com/MrDavidAlv/Robot4WD4WS)

</div>

> Autonomous 4WD4WS robotic platform with full navigation, SLAM, and omnidirectional kinematics built on ROS2 Humble. Designed for indoor material transport in university environments.

---

## Robot Gallery

<div align="center">
<table>
  <tr>
    <td><img src="images/robot1.jpg" width="380"/></td>
    <td><img src="images/robot2.jpg" width="380"/></td>
  </tr>
  <tr>
    <td><img src="images/robot3.jpg" width="380"/></td>
    <td><img src="images/robot4.jpg" width="380"/></td>
  </tr>
</table>
</div>

---

## System Architecture

<div align="center">
<table>
  <tr>
    <th>Transform Tree (TF)</th>
    <th>SLAM System</th>
  </tr>
  <tr>
    <td><img src="images/URDF-TF.png" width="480"/></td>
    <td><img src="images/SLAM.png" width="480"/></td>
  </tr>
  <tr>
    <th>Navigation (Nav2)</th>
    <th>Mathematical Model</th>
  </tr>
  <tr>
    <td><img src="images/Navigation.png" width="480"/></td>
    <td><img src="images/modelo.png" width="480"/></td>
  </tr>
</table>
</div>

---

## Packages and Launch Files

| Package | Launch File | Description |
|---------|-------------|-------------|
| `tadeocar_bringup` | `slam_bringup.launch.py` | Full simulation with SLAM (Gazebo + kinematics + SLAM Toolbox + RViz2) |
| `tadeocar_bringup` | `navigation_bringup.launch.py` | Full simulation with Nav2 (Gazebo + kinematics + Nav2 + RViz2) |
| `tadeocar_control` | `control.launch.py` | Kinematics node only |
| `tadeocar_control` | `fourws_control.launch.py` | 4WS kinematics with joint state publishing |
| `tadeocar_control` | `xbox_control.launch.py` | Kinematics + Xbox controller |
| `tadeocar_description` | `display.launch.py` | URDF/TF visualization in RViz2 |
| `tadeocar_gazebo` | `simulation.launch.py` | Gazebo Fortress simulation only |
| `tadeocar_navigation` | `navigation.launch.py` | Nav2 stack (AMCL + global planner + DWB) |
| `tadeocar_slam` | `slam.launch.py` | SLAM Toolbox (async mapping) |
| `tadeocar_slam` | `save_map.launch.py` | Save the generated map |

---

## Installation

### Dependencies

```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros-gz \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher

pip3 install websockets numpy
```

### Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

```bash
# SLAM mapping
ros2 launch tadeocar_bringup slam_bringup.launch.py

# Save map
ros2 launch tadeocar_slam save_map.launch.py

# Autonomous navigation
ros2 launch tadeocar_bringup navigation_bringup.launch.py

# Model visualization
ros2 launch tadeocar_description display.launch.py
```

Maps are saved to `src/tadeocar_navigation/maps/`.

---

<div align="center">
Developed by the Semillero Robotica Utadeo
</div>
