# TadeoeCar 4WD4WS Robot

Robot móvil con **4 ruedas motrices y 4 ruedas direccionales** (4WD4WS) para Gazebo Clásico con ROS2 Humble.

## 🚗 Características

- **Cinemática avanzada:** Omnidireccional, Ackermann, y Cangrejo
- **Control dual:**
  - Xbox Controller (joysticks izquierdo/derecho)
  - WebSocket (interfaz web)
- **SLAM:** Mapeo en tiempo real con SLAM Toolbox
- **NAV2:** Navegación autónoma
- **Sensores:** LIDAR 360° (compatible con Axioma)

---

## 📁 Estructura del Proyecto

```
TadeoeCar-ws/
├── src/
│   ├── tadeocar_description/    # Modelo del robot, mundos, URDF, SDF
│   ├── tadeocar_control/        # Control cinemático, Xbox, WebSocket
│   ├── tadeocar_navigation/     # Configuración SLAM y NAV2
│   └── tadeocar_bringup/        # Launch files principales (⭐ NUEVO)
├── DEVELOPMENT_LOG.md           # Log detallado de desarrollo
└── README.md                    # Este archivo
```

### Paquetes

#### tadeocar_description
Modelo del robot y simulación:
- `models/tadeocar_v1/model.sdf` - Modelo físico del robot
- `urdf/tadeocar_tf.urdf` - Transformaciones TF
- `worlds/tadeocar.world` - Mundo de Gazebo

#### tadeocar_control
Nodos de control:
- `fourws_kinematics_node.py` - Control cinemático 4WD4WS
- `xbox_control_node.py` - Control con Xbox controller
- `gazebo_effort_bridge.py` - Puente Gazebo-ROS2
- `web_control_node.py` - Control websocket

#### tadeocar_navigation
Configuración de navegación:
- `config/slam_params.yaml` - Parámetros SLAM Toolbox
- `config/nav2_params.yaml` - Parámetros NAV2

#### tadeocar_bringup ⭐
Launch files organizados:
- `gazebo.launch.py` - Solo Gazebo con robot
- `control_bringup.launch.py` - Gazebo + Xbox control
- `slam_bringup.launch.py` - Sistema SLAM completo
- `save_map.launch.py` - Guardar mapa actual

---

## 🛠️ Instalación

### Prerrequisitos

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Gazebo Classic
sudo apt install gazebo libgazebo-dev

# Dependencias ROS2
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-joy \
  ros-humble-teleop-twist-joy

# Python dependencies
pip3 install websockets
```

### Compilación

```bash
cd ~/ros2/TadeoeCar-ws
colcon build
source install/setup.bash
```

---

## 🎮 Uso

### 1. Solo Gazebo (Sin Control)

Lanza únicamente Gazebo con el robot:

```bash
source install/setup.bash
ros2 launch tadeocar_bringup gazebo.launch.py
```

Controla manualmente con topics:
```bash
# Cambiar modo
ros2 topic pub /robot_mode std_msgs/msg/String "data: 'omnidirectional'"

# Mover robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 2. Simulación con Xbox Controller ⭐

**Conexión del control:**
```bash
# Verificar que el control esté conectado
ls /dev/input/js0

# Si no aparece, conectar el control por USB o Bluetooth
```

**Lanzar simulación:**
```bash
source install/setup.bash
ros2 launch tadeocar_bringup control_bringup.launch.py
```

**Controles Xbox:**

| Control | Función |
|---------|---------|
| **Joystick izquierdo** | Movimiento omnidireccional (XY) |
| **Joystick derecho** | Movimiento cangrejo (lateral) |
| **Botón A** | Cambiar a modo Omnidireccional |
| **Botón B** | Cambiar a modo Ackermann |
| **Botón X** | Cambiar a modo Cangrejo |

**Modos de movimiento:**

- **Omnidireccional:** Todas las ruedas apuntan en dirección del movimiento (360°)
- **Ackermann:** Solo ruedas delanteras se dirigen (conducción tipo auto)
- **Cangrejo:** Movimiento lateral puro

### 3. SLAM (Mapeo en Tiempo Real) ⭐

**Sistema completo de SLAM:**

```bash
source install/setup.bash
ros2 launch tadeocar_bringup slam_bringup.launch.py
```

**Componentes lanzados:**
- Gazebo con robot
- Xbox controller
- Control cinemático 4WD4WS
- SLAM Toolbox
- RViz2 (visualización)

**Uso:**
1. Mueve el robot con el Xbox controller
2. Observa el mapa construyéndose en RViz2
3. Guarda el mapa cuando termines:

```bash
# Opción 1: Con launch file
ros2 launch tadeocar_bringup save_map.launch.py map_name:=mi_mapa

# Opción 2: Comando directo
ros2 run nav2_map_server map_saver_cli -f ~/mi_mapa
```

### 4. WebSocket Control (Interfaz Web)

```bash
# Terminal 1: Simulación
source install/setup.bash
ros2 launch tadeocar_description complete_sim.launch.py

# Terminal 2: Servidor Web
source install/setup.bash
ros2 launch tadeocar_control control.launch.py
```

Abre en navegador: `http://localhost:8080`

---

## 🔧 Configuración Avanzada

### Ajustar Velocidades Máximas

Edita `/src/tadeocar_control/tadeocar_control/fourws_kinematics_node.py`:

```python
self.max_linear_speed = 2.0  # m/s
self.max_angular_speed = 1.0  # rad/s
self.max_steering_angle = 0.5  # radians (~28.6°)
```

### Ajustar Ganancias PID

```python
self.kp_steering = 20.0  # Steering control gain
self.kp_wheel = 3.0      # Wheel control gain
```

### Parámetros SLAM

Edita `/src/tadeocar_navigation/config/slam_params.yaml`

Parámetros importantes:
- `resolution: 0.05` - Resolución del mapa (metros/pixel)
- `max_laser_range: 3.5` - Rango máximo del LIDAR
- `minimum_travel_distance: 0.5` - Distancia mínima para actualizar

---

## 📊 Arquitectura del Sistema

```
┌─────────────┐      ┌──────────────┐      ┌─────────────────┐
│ Xbox / Web  │─────▶│ cmd_vel      │─────▶│ 4WS Kinematics  │
│ Controller  │      │ robot_mode   │      │ Node            │
└─────────────┘      └──────────────┘      └────────┬────────┘
                                                     │
                                                     ▼
                                          ┌──────────────────┐
                                          │ 8 cmd_effort     │
                                          │ topics           │
                                          └────────┬─────────┘
                                                   │
                                                   ▼
                                          ┌──────────────────┐
                                          │ Gazebo Effort    │
                                          │ Bridge           │
                                          └────────┬─────────┘
                                                   │
                                                   ▼
                                          ┌──────────────────┐
                                          │ Gazebo Classic   │
                                          │ (Physics)        │
                                          └──────────────────┘
```

---

## 🐛 Troubleshooting

### Problema: Robot no se mueve

**Solución 1:** Verificar que Gazebo esté corriendo
```bash
gz topic -l | grep apply_joint_effort
```

**Solución 2:** Verificar topics ROS2
```bash
ros2 topic echo /cmd_vel
ros2 topic echo /joint_states
```

### Problema: Xbox controller no detectado

```bash
# Verificar dispositivo
ls -la /dev/input/js*

# Probar con jstest
sudo apt install joystick
jstest /dev/input/js0
```

### Problema: SLAM no mapea

```bash
# Verificar LIDAR
ros2 topic echo /scan

# Verificar TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Problema: Ruedas oscilan (jittering)

Reduce la ganancia PID en `fourws_kinematics_node.py`:
```python
self.kp_steering = 15.0  # Reducir de 20.0
```

---

## 📚 Documentación Adicional

- **DEVELOPMENT_LOG.md:** Log detallado del desarrollo con arquitectura completa
- **Fase 1:** Ajuste de sensores
- **Fase 2:** Plugin 4WD4WS para Gazebo Clásico
- **Fase 3:** Control Xbox
- **Fase 4:** SLAM y navegación

---

## 🎯 Modos de Operación

### Modo Omnidireccional
```bash
ros2 topic pub /robot_mode std_msgs/msg/String "data: 'omnidirectional'"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
- Todas las ruedas apuntan en dirección del movimiento
- Permite movimiento en cualquier dirección (360°)

### Modo Ackermann
```bash
ros2 topic pub /robot_mode std_msgs/msg/String "data: 'ackermann'"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```
- Solo ruedas delanteras se dirigen
- Conducción tipo automóvil tradicional

### Modo Cangrejo
```bash
ros2 topic pub /robot_mode std_msgs/msg/String "data: 'crab'"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
- Todas las ruedas al mismo ángulo
- Movimiento lateral puro

---

## 🚀 Comandos Rápidos

```bash
# Compilar todo
cd ~/ros2/TadeoeCar-ws && colcon build && source install/setup.bash

# ⭐ RECOMENDADOS (usando tadeocar_bringup)
# Solo Gazebo
ros2 launch tadeocar_bringup gazebo.launch.py

# Gazebo + Xbox
ros2 launch tadeocar_bringup control_bringup.launch.py

# SLAM completo (Gazebo + Xbox + SLAM + RViz)
ros2 launch tadeocar_bringup slam_bringup.launch.py

# Guardar mapa
ros2 launch tadeocar_bringup save_map.launch.py map_name:=mi_mapa

# Otros comandos útiles
# Solo control Xbox (sin Gazebo)
ros2 launch tadeocar_control xbox_control.launch.py

# WebSocket
ros2 launch tadeocar_control control.launch.py
```

---

## 📈 Roadmap

- [x] Fase 1: Sensores compatibles con Axioma
- [x] Fase 2: Plugin 4WD4WS para Gazebo Clásico
- [x] Fase 3: Control Xbox dual joystick
- [x] Fase 4: SLAM Toolbox
- [ ] Fase 5: NAV2 completo
- [ ] Fase 6: WebSocket adaptado a cmd_vel
- [ ] Fase 7: Optimización y pruebas

---

## 📝 Licencia

Apache 2.0 (tadeocar_description)
MIT (tadeocar_control)

---

## 👥 Autores

- **Axioma Robotics**
- Proyecto TadeoeCar 4WD4WS

---

## 🔗 Referencias

- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Gazebo Classic](http://gazebosim.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [NAV2](https://navigation.ros.org/)

---

**Última actualización:** 2025-11-02
