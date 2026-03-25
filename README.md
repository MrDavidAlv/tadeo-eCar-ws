# Tadeo eCar 4WD4WS Autonomous Robot

<div align="center">
<img src="images/portada.png" width="280"/>

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](#)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](#)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-orange)](#)
[![Nav2](https://img.shields.io/badge/Nav2-Humble-00599C)](#)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-green)](#)
[![RTAB--Map](https://img.shields.io/badge/RTAB--Map-Visual%20SLAM-8B5CF6)](#)

</div>

> Autonomous 4WD4WS robotic platform with LiDAR SLAM, Visual SLAM (RTAB-Map + ZED2i), and omnidirectional kinematics. Built on ROS2 Humble for indoor material transport.

---

## Robot

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

## Package Architecture

<div align="center">
<img src="images/architecture-packages.svg" width="800"/>
</div>

---

## Control Flow

<div align="center">
<img src="images/control-flow.svg" width="800"/>
</div>

---

## Sensor Pipeline

<div align="center">
<img src="images/sensor-pipeline.svg" width="800"/>
</div>

---

## Hardware Architecture (Real Robot)

<div align="center">
<img src="images/hardware-architecture.svg" width="850"/>
</div>

---

## TF Frame Tree

<div align="center">
<img src="images/tf-tree.svg" width="700"/>
</div>

---

## Simulation Screenshots

<div align="center">
<table>
  <tr>
    <th>SLAM</th>
    <th>Navigation</th>
  </tr>
  <tr>
    <td><img src="images/SLAM.png" width="450"/></td>
    <td><img src="images/Navigation.png" width="450"/></td>
  </tr>
</table>
</div>

---

## Specifications

| Parameter | Simulation | Real Robot |
|-----------|-----------|------------|
| Wheelbase | 1.058 m | 1.058 m |
| Track width | 0.55 m | 0.55 m |
| Steering | 4WS | 4x Industrial servos 270 deg |
| Drive | 4WD | 4x BLDC (2x ODrive S3) |
| Compute | Host PC | Jetson Nano 4GB |
| Camera | ZED2i (simulated) | ZED2i (USB 3.0) |
| LiDAR | 2D, 360 deg, 3.5m | YDLidar X2 (8m) |
| Power | N/A | 2x LiPo 24V |

---

## Installation

```bash
# Dependencies
sudo apt install -y \
  ros-humble-ros-gz ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-rtabmap-ros ros-humble-twist-mux \
  ros-humble-joy ros-humble-teleop-twist-joy ros-humble-rviz2 \
  ros-humble-xacro ros-humble-robot-state-publisher

pip3 install websockets numpy

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

```bash
# LiDAR SLAM
ros2 launch tadeocar_bringup slam_bringup.launch.py

# Visual SLAM (RTAB-Map + ZED2i)
ros2 launch tadeocar_bringup vslam_bringup.launch.py

# Autonomous navigation
ros2 launch tadeocar_bringup navigation_bringup.launch.py

# Xbox controller
ros2 launch tadeocar_control xbox_control.launch.py

# Save map
ros2 launch tadeocar_slam save_map.launch.py
```

---

## Documentation

Technical documentation and mathematical model in [documentacion/](documentacion/).

---

<div align="center">
Semillero de Robotica - Universidad de Bogota Jorge Tadeo Lozano
</div>
