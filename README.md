# 🤖 Robot Autónomo Tadeo eCar 4WD4WS

<div align="center">
<img src="images/portada.png" width="300"/>


[![Lenguaje Python](https://img.shields.io/badge/Python-3.8+-yellow?logo=python)](#)
[![Sistema Operativo](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](#)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](#)
[![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress-orange?logo=gazebo)](#)
[![Colcon](https://img.shields.io/badge/Build-Colcon-22314E)](#)
[![Nav2](https://img.shields.io/badge/Nav2-Humble-00599C)](#)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-green)](#)
[![Git](https://img.shields.io/badge/Git-2.34+-F05032?logo=git)](#)
[![VS Code](https://img.shields.io/badge/IDE-VS%20Code-007ACC?logo=visualstudiocode)](#)
[![Shell](https://img.shields.io/badge/Shell-Bash-4EAA25?logo=gnubash)](#)
[![Arquitectura](https://img.shields.io/badge/CPU-x86_64%20%7C%20ARM64-lightgrey?logo=amd)](#)
[![Versión Actual](https://img.shields.io/badge/Versión-v1.0.0-blue)](#)
[![Repositorio](https://img.shields.io/badge/GitHub-MrDavidAlv-181717?logo=github)](https://github.com/MrDavidAlv/Robot4WD4WS)

</div>

> Plataforma robótica autónoma para logística universitaria en interiores. Robot eléctrico omnidireccional 4WD 4WS con capacidades de navegación autónoma, SLAM, visión computacional y planificación de trayectorias usando ROS2 Humble. Diseñado para transporte de materiales en ambientes universitarios.

---

## Características

- **Sistema de tracción 4WD4WS**: Cuatro ruedas motrices y direccionales independientes
- **Control omnidireccional**: Movilidad de 360° con múltiples modos de conducción
- **Navegación autónoma**: Stack completo de Nav2 para planificación y evasión de obstáculos
- **SLAM en tiempo real**: Mapeo y localización simultáneos con SLAM Toolbox
- **Cinemática avanzada**: Control de dirección y velocidad de las cuatro ruedas con modelos Ackermann, Omnidireccional y Crab
- **Control multi-interfaz**: Soporte para Xbox controller, interfaz web y comandos de navegación
- **Simulación completa**: Integración con Gazebo Classic y ros2_control
- **Sensor LiDAR**: Rango de 3.5m, 320 muestras, 20Hz para percepción del entorno

---

## Galería del Robot

<div align="center">
<table>
  <tr>
    <td><img src="images/robot1.jpg" width="400"/></td>
    <td><img src="images/robot2.jpg" width="400"/></td>
  </tr>
  <tr>
    <td><img src="images/robot3.jpg" width="400"/></td>
    <td><img src="images/robot4.jpg" width="400"/></td>
  </tr>
</table>
</div>

---

## Arquitectura del Sistema

### Transformadas (TF Tree)
<div align="center">
<img src="images/URDF-TF.png" width="800"/>
</div>

El árbol de transformadas define las relaciones espaciales entre todos los componentes del robot desde `base_footprint` hasta el sensor LiDAR en `base_scan`.

### Sistema SLAM
<div align="center">
<img src="images/SLAM.png" width="800"/>
</div>

SLAM Toolbox genera mapas de ocupación en tiempo real procesando datos del LiDAR y odometría, permitiendo localización simultánea en entornos desconocidos.

### Sistema de Navegación
<div align="center">
<img src="images/Navigation.png" width="800"/>
</div>

Nav2 stack proporciona planificación global con NavFn, control local con DWB, costmaps dinámicos y comportamientos de recuperación para navegación autónoma robusta.

---

## Modelo Matemático

<div align="center">
<img src="images/modelo.png" width="800"/>
</div>

El robot Tadeo eCar cuenta con un modelo matemático completo que describe su cinemática, dinámica y sistema de control PID. El diagrama superior muestra la arquitectura completa del sistema: geometría del robot 4WD4WS, ecuaciones de cinemática inversa para los tres modos de operación, sistema de control PID y especificaciones técnicas.

### Documentación Técnica Completa


## Requisitos del Sistema

### Software Base
- **Sistema Operativo**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS2**: Humble Hawksbill
- **Gazebo**: Classic 11
- **Python**: 3.8 o superior
- **CMake**: 3.16 o superior

### Dependencias ROS2

#### Paquetes Core
```bash
ros-humble-ros-base
ros-humble-ros2-control
ros-humble-ros2-controllers
ros-humble-gazebo-ros-pkgs
ros-humble-gazebo-ros2-control
ros-humble-robot-state-publisher
ros-humble-joint-state-publisher
```

#### Navegación y SLAM
```bash
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-slam-toolbox
ros-humble-robot-localization
```

#### Control y Teleoperation
```bash
ros-humble-joy
ros-humble-teleop-twist-joy
ros-humble-rviz2
ros-humble-xacro
```

#### Mensajes y Interfaces
```bash
ros-humble-std-msgs
ros-humble-sensor-msgs
ros-humble-geometry-msgs
ros-humble-nav-msgs
ros-humble-tf2-ros
```

### Dependencias Python
```bash
websockets>=10.0
numpy>=1.21.0
```

---

## Instalación

### 1. Instalar ROS2 Humble

```bash
# Configurar locale UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Agregar repositorio ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 2. Instalar Dependencias del Proyecto

```bash
# Dependencias ROS2
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
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

# Dependencias Python
pip3 install websockets numpy
```

### 3. Clonar el Repositorio

```bash
# Crear workspace
mkdir -p ~/ros2/TadeoeCar-ws
cd ~/ros2/TadeoeCar-ws

# Clonar (o copiar el proyecto existente)
# git clone https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws.git .
```

---

## Compilación

### Compilar todos los paquetes

```bash
# Configurar entorno ROS2
source /opt/ros/humble/setup.bash

# Compilar workspace
cd ~/ros2/TadeoeCar-ws
colcon build --symlink-install

# Source el workspace
source install/setup.bash
```

### Compilar paquete específico

```bash
# Compilar solo tadeocar_control
colcon build --packages-select tadeocar_control

# Compilar control y navegación
colcon build --packages-select tadeocar_control tadeocar_navigation
```

### Limpiar build anterior

```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## Ejecución

### Modo SLAM (Mapeo)

Ejecuta simulación completa con Gazebo, control Xbox, cinemática 4WS y SLAM Toolbox para crear mapas:

```bash
source install/setup.bash
ros2 launch tadeocar_bringup slam.launch.py
```

**Componentes lanzados**:
- Gazebo Classic con mundo de prueba
- Nodo de cinemática `fourws_kinematics`
- Nodo de control Xbox `xbox_control`
- SLAM Toolbox (async)
- RViz2 con configuración SLAM

**Control**: Usa Xbox controller con RB para acelerar, joysticks para dirección.

### Guardar Mapa

Una vez completado el mapeo:

```bash
ros2 launch tadeocar_bringup save_map.launch.py
```

Los mapas se guardan en `src/tadeocar_navigation/maps/` como `mapa.pgm` y `mapa.yaml`.

### Modo Navegación Autónoma

Ejecuta navegación autónoma con Nav2 usando un mapa previamente generado:

```bash
source install/setup.bash
ros2 launch tadeocar_bringup navigation.launch.py
```

**Componentes lanzados**:
- Gazebo Classic
- Nodo de cinemática `fourws_kinematics`
- Stack completo de Nav2 (AMCL, planificador, controlador)
- RViz2 con herramientas de navegación

**Uso**: Define `2D Goal Pose` en RViz2 para enviar objetivos de navegación.

### Solo Control y Cinemática

Para desarrollo o pruebas sin SLAM/Nav2:

```bash
# Solo cinemática
ros2 launch tadeocar_control fourws_control.launch.py

# Control Xbox + Cinemática
ros2 launch tadeocar_control xbox_control.launch.py
```

### Visualización con RViz2

```bash
# Visualización de TF y modelo
ros2 launch tadeocar_description display.launch.py

# Visualización SLAM
rviz2 -d src/tadeocar_navigation/config/slam.rviz

# Visualización Navegación
rviz2 -d src/tadeocar_navigation/config/navigation.rviz
```

---

<div align="center">
Desarrollado con ❤️ por Tadeo Robotics Group
</div>
