# Robot Autónomo Tadeo eCar 4WD4WS

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

> Plataforma robótica autónoma 4WD4WS con navegación autónoma, SLAM, y cinemática omnidireccional usando ROS2 Humble. Diseñada para transporte de materiales en entornos universitarios.

---

## Galería del Robot

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

## Arquitectura del Sistema

<div align="center">
<table>
  <tr>
    <th>Transformadas (TF Tree)</th>
    <th>Sistema SLAM</th>
  </tr>
  <tr>
    <td><img src="images/URDF-TF.png" width="480"/></td>
    <td><img src="images/SLAM.png" width="480"/></td>
  </tr>
  <tr>
    <th>Navegación (Nav2)</th>
    <th>Modelo Matemático</th>
  </tr>
  <tr>
    <td><img src="images/Navigation.png" width="480"/></td>
    <td><img src="images/modelo.png" width="480"/></td>
  </tr>
</table>
</div>

---

## Paquetes y Launch Files

| Paquete | Launch File | Descripción |
|---------|-------------|-------------|
| `tadeocar_bringup` | `slam_bringup.launch.py` | Simulación completa con SLAM (Gazebo + cinemática + SLAM Toolbox + RViz2) |
| `tadeocar_bringup` | `navigation_bringup.launch.py` | Simulación completa con Nav2 (Gazebo + cinemática + Nav2 + RViz2) |
| `tadeocar_control` | `control.launch.py` | Solo nodo de cinemática 4WS |
| `tadeocar_control` | `fourws_control.launch.py` | Cinemática 4WS con publicación de estado |
| `tadeocar_control` | `xbox_control.launch.py` | Cinemática + control Xbox |
| `tadeocar_description` | `display.launch.py` | Visualización URDF/TF en RViz2 |
| `tadeocar_gazebo` | `simulation.launch.py` | Solo simulación Gazebo Fortress |
| `tadeocar_navigation` | `navigation.launch.py` | Stack Nav2 (AMCL + planificador + DWB) |
| `tadeocar_slam` | `slam.launch.py` | SLAM Toolbox (mapeo async) |
| `tadeocar_slam` | `save_map.launch.py` | Guardar mapa generado |

---

## Instalación

### Dependencias

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

### Compilar

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Ejecución

```bash
# Mapeo con SLAM
ros2 launch tadeocar_bringup slam_bringup.launch.py

# Guardar mapa
ros2 launch tadeocar_slam save_map.launch.py

# Navegación autónoma
ros2 launch tadeocar_bringup navigation_bringup.launch.py

# Visualización del modelo
ros2 launch tadeocar_description display.launch.py
```

Los mapas se guardan en `src/tadeocar_navigation/maps/`.

---

<div align="center">
Desarrollado por el Semillero Robotica Utadeo
</div>
