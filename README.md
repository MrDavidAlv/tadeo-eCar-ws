# TadeoeCar-ws - Robot 4WS4WD

Robot móvil 4WS4WD (cuatro ruedas con dirección y tracción independiente) usando modelo SDF para ROS2 Humble y Gazebo Classic 11.10.2.

## Requisitos

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Compilación

```bash
cd ~/ros2/TadeoeCar-ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Uso

**IMPORTANTE:** Si Gazebo no inicia, cierra procesos anteriores:
```bash
killall -9 gzserver gzclient
```

### Lanzar Gazebo

**Con GUI:**
```bash
ros2 launch tadeocar_description gazebo.launch.py
```

**Sin GUI (para máquinas limitadas):**
```bash
ros2 launch tadeocar_description gazebo.launch.py gui:=false
```

El robot se spawneará automáticamente desde el modelo SDF.

### Insertar modelo manualmente en Gazebo

También puedes abrir Gazebo y usar el panel "Insert" para añadir el modelo `tadeocar_v1`.

## Tópicos

- `/joint_states` - Estados de todas las articulaciones (8 joints)
- `/scan` - LiDAR 2D (360 samples, 10Hz)
- `/camera/image_raw` - Cámara (640x480, 10Hz)
- `/camera/camera_info` - Info de cámara
- `/imu` - IMU (100Hz)

## Especificaciones

- **Dimensiones**: 1.2m x 0.8m x 0.4m
- **Wheelbase**: 1.058 m
- **Track width**: 0.55 m
- **Radio rueda**: 0.1 m
- **Masa total**: ~90 kg

### Límites

- Velocidad ruedas: 100 rad/s max
- Ángulo dirección: -0.5 a 0.5 rad (±28.6°)
- Esfuerzo dirección: 100 N·m
- Esfuerzo ruedas: 10 N·m

## Estructura

```
src/tadeocar_description/
├── launch/
│   └── gazebo.launch.py
├── models/
│   └── tadeocar_v1/
│       ├── model.config
│       ├── model.sdf
│       └── meshes/
│           ├── chassis/
│           ├── suspension/
│           └── wheels/
└── worlds/
    └── tadeocar.world
```

## Control del Robot

El robot tiene 8 joints:
- 4 steering joints (dirección): `*_steering_joint`
- 4 wheel joints (tracción): `*_wheel_joint`

Para controlar, publica en los topics de comando de cada joint.

## Sensores

- **LiDAR**: Montado en el centro superior (z=0.43m)
- **Cámara**: Montada en el frente (x=0.6m, z=0.38m)
- **IMU**: Ubicado en el centro del base_link

## Licencia

Apache-2.0
