# TadeoeCar 4WD4WS - Guía Completa de Pruebas

**Fecha:** 2025-11-02
**Versión:** 1.0
**Robot:** TadeoeCar 4WD4WS

---

## 📋 ÍNDICE DE PRUEBAS

1. [Pruebas Preliminares](#1-pruebas-preliminares)
2. [Prueba de Sensores](#2-prueba-de-sensores)
3. [Prueba de Movimiento Básico](#3-prueba-de-movimiento-básico)
4. [Prueba con Xbox Controller](#4-prueba-con-xbox-controller)
5. [Prueba con WebSocket](#5-prueba-con-websocket)
6. [Prueba de SLAM](#6-prueba-de-slam)
7. [Prueba de Navegación (NAV2)](#7-prueba-de-navegación-nav2)

---

## 1. PRUEBAS PRELIMINARES

### 1.1 Verificar Compilación

```bash
cd ~/ros2/TadeoeCar-ws
colcon build
```

**Resultado esperado:**
```
Starting >>> tadeocar_description
Starting >>> tadeocar_control
Starting >>> tadeocar_navigation
Starting >>> tadeocar_bringup
Finished <<< tadeocar_description [0.3s]
Finished <<< tadeocar_control [1.0s]
Finished <<< tadeocar_navigation [0.7s]
Finished <<< tadeocar_bringup [0.7s]

Summary: 4 packages finished [~3s]
```

**✅ PASS:** Si todos los paquetes compilan sin errores
**❌ FAIL:** Si hay errores de compilación → revisar dependencias

---

### 1.2 Verificar Paquetes Instalados

```bash
source install/setup.bash

# Verificar que los paquetes están disponibles
ros2 pkg list | grep tadeo
```

**Resultado esperado:**
```
tadeocar_bringup
tadeocar_control
tadeocar_description
tadeocar_navigation
```

**✅ PASS:** Si aparecen los 4 paquetes
**❌ FAIL:** Si falta alguno → volver a compilar

---

### 1.3 Verificar Launch Files

```bash
# Verificar launch files de bringup
ros2 launch tadeocar_bringup --show-args gazebo.launch.py
ros2 launch tadeocar_bringup --show-args control_bringup.launch.py
ros2 launch tadeocar_bringup --show-args slam_bringup.launch.py
```

**Resultado esperado:**
```
Arguments (pass arguments as '<name>:=<value>'):
    'use_sim_time':
        Use simulation time
        (default: 'true')
    'gui':
        Start Gazebo GUI
        (default: 'true')
```

**✅ PASS:** Si muestra los argumentos
**❌ FAIL:** Si da error → revisar instalación

---

## 2. PRUEBA DE SENSORES

### 2.1 Lanzar Gazebo Solo

**Terminal 1:**
```bash
cd ~/ros2/TadeoeCar-ws
source install/setup.bash
ros2 launch tadeocar_bringup gazebo.launch.py
```

**Resultado esperado:**
- Se abre Gazebo GUI
- Robot TadeoeCar aparece en el mundo
- Robot está suspendido a 0.5m del suelo
- No hay errores en terminal

**⏱️ Tiempo:** ~10-15 segundos para cargar

**✅ PASS:** Si Gazebo se abre y robot aparece
**❌ FAIL:** Si Gazebo crashea → revisar instalación de Gazebo

---

### 2.2 Verificar LIDAR

**Terminal 2 (mantener Terminal 1 abierto):**
```bash
source install/setup.bash

# Verificar que el topic /scan existe
ros2 topic list | grep scan
```

**Resultado esperado:**
```
/scan
```

**Verificar datos del LIDAR:**
```bash
ros2 topic info /scan
```

**Resultado esperado:**
```
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 0
```

**Ver datos en tiempo real:**
```bash
ros2 topic echo /scan --once
```

**Resultado esperado:**
```yaml
header:
  stamp:
    sec: 123
    nanosec: 456789000
  frame_id: base_scan
angle_min: 0.0
angle_max: 6.28
angle_increment: 0.0174532  # ~1 grado
time_increment: 0.0
scan_time: 0.05
range_min: 0.12
range_max: 3.5
ranges: [3.5, 3.5, 3.5, ...]  # 360 valores
intensities: []
```

**Verificar frecuencia:**
```bash
ros2 topic hz /scan
```

**Resultado esperado:**
```
average rate: 20.000
	min: 0.049s max: 0.051s std dev: 0.001s window: 100
```

**✅ PASS:** Si recibe 20 Hz con 360 muestras
**❌ FAIL:** Si no hay datos → verificar plugin en model.sdf

---

### 2.3 Verificar Joint States

```bash
# Verificar topic
ros2 topic list | grep joint

# Debería aparecer:
# /joint_states

# Ver datos
ros2 topic echo /joint_states --once
```

**Resultado esperado:**
```yaml
header:
  stamp: ...
  frame_id: ''
name:
- front_left_steering_joint
- front_left_wheel_joint
- front_right_steering_joint
- front_right_wheel_joint
- rear_left_steering_joint
- rear_left_wheel_joint
- rear_right_steering_joint
- rear_right_wheel_joint
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: []
```

**✅ PASS:** Si aparecen los 8 joints
**❌ FAIL:** Si faltan joints → verificar model.sdf

---

### 2.4 Verificar TF Tree

```bash
# Verificar transformaciones
ros2 run tf2_tools view_frames

# Esperar ~5 segundos, luego:
evince frames.pdf
```

**Resultado esperado en frames.pdf:**
```
base_footprint
  └─ base_link
       └─ base_scan
```

**✅ PASS:** Si el árbol TF es correcto
**❌ FAIL:** Si faltan frames → verificar robot_state_publisher

---

### 2.5 Verificar Visualización en RViz

```bash
# Terminal 3
source install/setup.bash
rviz2
```

**En RViz:**
1. Click en "Add" (abajo izquierda)
2. Agregar "LaserScan"
3. Configurar:
   - Topic: `/scan`
   - Fixed Frame: `base_scan`
4. Debería verse un círculo de puntos LIDAR

**✅ PASS:** Si se ve el LIDAR en RViz
**❌ FAIL:** Si no aparece → verificar frame_id

---

## 3. PRUEBA DE MOVIMIENTO BÁSICO

### 3.1 Lanzar Sistema de Control

**Cerrar procesos anteriores (Ctrl+C en todas las terminales)**

**Terminal 1:**
```bash
cd ~/ros2/TadeoeCar-ws
source install/setup.bash
ros2 launch tadeocar_bringup control_bringup.launch.py
```

**Resultado esperado:**
- Gazebo se abre con robot
- Nodos de control se lanzan:
  - `fourws_kinematics_node`
  - `gazebo_effort_bridge`
  - (joy_node y xbox_control_node también pero pueden fallar si no hay Xbox)

**⏱️ Tiempo:** ~15-20 segundos

---

### 3.2 Verificar Nodos Activos

**Terminal 2:**
```bash
source install/setup.bash
ros2 node list
```

**Resultado esperado:**
```
/fourws_kinematics_node
/gazebo
/gazebo_effort_bridge
/gazebo_ros_joint_state_publisher
/joy_node  (puede fallar sin Xbox)
/robot_state_publisher
/xbox_control_node  (puede fallar sin Xbox)
```

**✅ PASS:** Si aparecen los nodos principales (fourws_kinematics y gazebo_bridge)
**⚠️ WARNING:** joy_node y xbox_control_node fallarán sin Xbox conectado (normal)

---

### 3.3 Verificar Topics de Control

```bash
ros2 topic list | grep -E "(cmd_vel|robot_mode|cmd_effort)"
```

**Resultado esperado:**
```
/cmd_vel
/robot_mode
/model/tadeocar_v1/joint/front_left_steering_joint/cmd_effort
/model/tadeocar_v1/joint/front_left_wheel_joint/cmd_effort
/model/tadeocar_v1/joint/front_right_steering_joint/cmd_effort
/model/tadeocar_v1/joint/front_right_wheel_joint/cmd_effort
/model/tadeocar_v1/joint/rear_left_steering_joint/cmd_effort
/model/tadeocar_v1/joint/rear_left_wheel_joint/cmd_effort
/model/tadeocar_v1/joint/rear_right_steering_joint/cmd_effort
/model/tadeocar_v1/joint/rear_right_wheel_joint/cmd_effort
```

**✅ PASS:** Si aparecen todos los topics
**❌ FAIL:** Si faltan → verificar que nodos se lanzaron

---

### 3.4 Prueba: Mover Hacia Adelante (Modo Omnidireccional)

**Terminal 2:**
```bash
# Asegurar modo omnidireccional
ros2 topic pub --once /robot_mode std_msgs/msg/String "data: 'omnidirectional'"

# Mover hacia adelante (1 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

**Observar en Gazebo:**
- Robot debería moverse hacia adelante
- Las 4 ruedas deben apuntar hacia adelante (ángulo ~0°)
- Robot se mueve en línea recta

**⏱️ Dejar correr 5-10 segundos**

**Detener:**
```bash
# Ctrl+C para detener publicación
# Luego detener robot:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**✅ PASS:** Si robot avanza en línea recta
**❌ FAIL:** Si robot no se mueve → verificar gazebo_effort_bridge

---

### 3.5 Prueba: Movimiento Lateral (Omnidireccional)

```bash
# Mover lateralmente a la derecha
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

**Observar en Gazebo:**
- Las 4 ruedas deben girar 90° (perpendicular)
- Robot se mueve lateralmente (como cangrejo)

**⏱️ Dejar correr 5 segundos**

**Detener:**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**✅ PASS:** Si robot se mueve lateralmente
**❌ FAIL:** Si ruedas no giran → verificar PID en fourws_kinematics_node

---

### 3.6 Prueba: Movimiento Diagonal

```bash
# Mover en diagonal (45°)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

**Observar:**
- Las 4 ruedas deben girar ~45°
- Robot se mueve en diagonal

**Detener después de 5 segundos**

**✅ PASS:** Si robot se mueve en diagonal
**❌ FAIL:** Si dirección es incorrecta → revisar cinemática

---

### 3.7 Prueba: Rotación en el Lugar

```bash
# Girar en el lugar
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 10
```

**Observar:**
- Ruedas en patrón diamante (45°, -45°, -45°, 45°)
- Robot gira sobre su eje central sin desplazarse

**Detener después de 5 segundos**

**✅ PASS:** Si robot gira en su lugar
**❌ FAIL:** Si se desplaza → ajustar cinemática de spin

---

### 3.8 Prueba: Modo Ackermann

```bash
# Cambiar a modo Ackermann
ros2 topic pub --once /robot_mode std_msgs/msg/String "data: 'ackermann'"

# Avanzar girando (como auto)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" -r 10
```

**Observar:**
- Solo ruedas DELANTERAS se dirigen
- Ruedas TRASERAS permanecen rectas
- Robot gira como un auto tradicional

**Detener después de 5 segundos**

**✅ PASS:** Si solo ruedas delanteras giran
**❌ FAIL:** Si todas las ruedas giran → verificar modo

---

### 3.9 Prueba: Modo Cangrejo (Crab)

```bash
# Cambiar a modo cangrejo
ros2 topic pub --once /robot_mode std_msgs/msg/String "data: 'crab'"

# Movimiento lateral
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

**Observar:**
- Las 4 ruedas deben girar al mismo ángulo (90°)
- Robot se mueve lateralmente

**Detener después de 5 segundos**

**✅ PASS:** Si movimiento es lateral puro
**❌ FAIL:** Similar a omnidireccional pero sin capacidad de rotación simultánea

---

### 3.10 Tabla Resumen de Pruebas de Movimiento

| Modo | Comando | Comportamiento Esperado |
|------|---------|------------------------|
| Omnidireccional | `x=1, y=0` | Avanza recto, todas las ruedas 0° |
| Omnidireccional | `x=0, y=1` | Lateral derecha, todas las ruedas 90° |
| Omnidireccional | `x=1, y=1` | Diagonal, todas las ruedas 45° |
| Omnidireccional | `wz=0.5` | Giro en lugar, ruedas en diamante |
| Ackermann | `x=1, wz=0.3` | Solo ruedas delanteras giran |
| Crab | `x=0, y=1` | Lateral, todas las ruedas 90° |

---

## 4. PRUEBA CON XBOX CONTROLLER

### 4.1 Conectar Xbox Controller

**USB:**
```bash
# Conectar Xbox controller por USB
# Verificar dispositivo
ls -la /dev/input/js0
```

**Bluetooth (si aplica):**
```bash
bluetoothctl
# > scan on
# > pair XX:XX:XX:XX:XX:XX
# > connect XX:XX:XX:XX:XX:XX
# > trust XX:XX:XX:XX:XX:XX
# > exit
```

**Resultado esperado:**
```
crw-rw---- 1 root input 13, 0 Nov  2 10:00 /dev/input/js0
```

**✅ PASS:** Si aparece /dev/input/js0
**❌ FAIL:** Si no aparece → verificar conexión

---

### 4.2 Probar Joystick

```bash
# Instalar jstest si no está
sudo apt install joystick

# Probar joystick
jstest /dev/input/js0
```

**Resultado esperado:**
- Mueve los joysticks y debe cambiar valores
- Presiona botones y debe cambiar estados
- Ejes: 0-5
- Botones: 0-10+

**Presiona Ctrl+C para salir**

**✅ PASS:** Si detecta movimientos
**❌ FAIL:** Si no responde → verificar batería/conexión

---

### 4.3 Lanzar Sistema con Xbox

**Cerrar procesos anteriores**

**Terminal 1:**
```bash
cd ~/ros2/TadeoeCar-ws
source install/setup.bash
ros2 launch tadeocar_bringup control_bringup.launch.py
```

**Resultado esperado:**
- Gazebo se abre
- joy_node se lanza SIN errores
- xbox_control_node se lanza sin errores
- Mensaje: "Xbox Control Node initialized"

**⏱️ Tiempo:** ~20 segundos

---

### 4.4 Verificar Joy Node

**Terminal 2:**
```bash
source install/setup.bash

# Verificar topic /joy
ros2 topic echo /joy
```

**Resultado esperado:**
- Al mover joysticks, valores cambian
- axes: [0.0, 0.0, 0.0, ...] cambian entre -1.0 y 1.0
- buttons: [0, 0, 0, ...] cambian a 1 cuando presionas

**Ejemplo:**
```yaml
header:
  stamp: ...
  frame_id: ''
axes: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # Joystick izq hacia arriba
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
```

**✅ PASS:** Si valores cambian al mover joystick
**❌ FAIL:** Si no hay datos → verificar joy_node

---

### 4.5 Prueba: Movimiento con Joystick Izquierdo

**Instrucciones:**
1. **Mover joystick izquierdo hacia ARRIBA** (eje vertical)
2. Observar Gazebo: robot avanza hacia adelante
3. **Mover joystick izquierdo a la DERECHA** (eje horizontal)
4. Observar: robot se mueve lateralmente a la derecha
5. **Mover en diagonal** (joystick izquierdo diagonal superior-derecha)
6. Observar: robot se mueve en diagonal

**Verificar en terminal:**
```bash
ros2 topic echo /cmd_vel
```

**Resultado esperado:**
```yaml
linear:
  x: 1.0  # Proporcional a joystick vertical
  y: 0.5  # Proporcional a joystick horizontal
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

**✅ PASS:** Si robot responde al joystick izquierdo
**❌ FAIL:** Si no se mueve → verificar xbox_control_node

---

### 4.6 Prueba: Movimiento con Joystick Derecho (Modo Cangrejo)

**Instrucciones:**
1. **Soltar joystick izquierdo** (volver a centro)
2. **Mover joystick derecho hacia ARRIBA**
3. Observar: robot cambia automáticamente a modo "crab"
4. Robot se mueve hacia adelante en modo cangrejo
5. **Mover joystick derecho a la DERECHA**
6. Robot se mueve lateralmente

**Verificar cambio de modo:**
```bash
ros2 topic echo /robot_mode
```

**Resultado esperado:**
```yaml
data: 'crab'
```

**✅ PASS:** Si cambia a modo crab y robot se mueve
**❌ FAIL:** Si no cambia modo → verificar lógica en xbox_control_node

---

### 4.7 Prueba: Botones de Modo

**Instrucciones:**

**Botón A:**
1. Presiona **botón A** (botón inferior)
2. Verificar modo: `ros2 topic echo /robot_mode`
3. Debería cambiar a: `data: 'omnidirectional'`

**Botón B:**
1. Presiona **botón B** (botón derecho)
2. Verificar modo cambió a: `data: 'ackermann'`

**Botón X:**
1. Presiona **botón X** (botón izquierdo)
2. Verificar modo cambió a: `data: 'crab'`

**Probar cada modo:**
- Modo omnidireccional: Mover joystick izquierdo
- Modo ackermann: Avanzar y girar (como auto)
- Modo crab: Movimiento lateral puro

**✅ PASS:** Si botones cambian modos correctamente
**❌ FAIL:** Si no cambia → verificar mapeo de botones

---

### 4.8 Prueba: Deadzone

**Instrucciones:**
1. Soltar joysticks (deberían volver a centro)
2. Robot debe detenerse automáticamente
3. Mover LIGERAMENTE el joystick (muy poco)
4. Robot NO debe moverse (deadzone activo)
5. Mover más el joystick (>10%)
6. Robot debe empezar a moverse

**✅ PASS:** Si pequeños movimientos son ignorados
**❌ FAIL:** Si robot es muy sensible → ajustar deadzone en xbox_control_node.py

---

## 5. PRUEBA CON WEBSOCKET

### 5.1 Estado Actual del WebSocket

**⚠️ IMPORTANTE:** El nodo `web_control_node.py` actual NO está adaptado para usar `/cmd_vel`. Usa el sistema antiguo de control directo por esfuerzo.

**Opciones:**

**Opción A: Probar sistema antiguo (actual)**
- Funciona pero NO usa el sistema nuevo de modos
- Control directo de joints

**Opción B: Adaptar primero (RECOMENDADO)**
- Requiere modificar `web_control_node.py`
- Tiempo estimado: 4-6 horas

---

### 5.2 Prueba del WebSocket Actual (Sistema Antiguo)

**Terminal 1:**
```bash
cd ~/ros2/TadeoeCar-ws
source install/setup.bash
ros2 launch tadeocar_bringup gazebo.launch.py
```

**Terminal 2:**
```bash
source install/setup.bash
ros2 launch tadeocar_control control.launch.py
```

**Resultado esperado:**
- Servidor WebSocket inicia en puerto 8765
- Servidor HTTP inicia en puerto 8080
- Mensaje: "TadeoeCar Web Control Node Started"

---

### 5.3 Acceder a Interfaz Web

**En navegador:**
```
http://localhost:8080
```

**Resultado esperado:**
- Página web carga con interfaz de control
- 4 botones de modo: Omnidireccional, Ackermann, Halo, Giro
- Joystick virtual visible
- Indicador de conexión: "CONECTADO" (verde)

**✅ PASS:** Si página carga y muestra "CONECTADO"
**❌ FAIL:** Si no carga → verificar puerto 8080

---

### 5.4 Probar Modos del WebSocket (Sistema Antiguo)

**⚠️ NOTA:** Estos modos usan control directo, NO pasan por fourws_kinematics_node.

#### Modo 1: Omnidireccional

**En la interfaz web:**
1. Click en botón **"Omnidireccional"**
2. Usar joystick virtual (arrastrar con mouse/touch)
3. Mover hacia arriba → robot avanza
4. Mover lateral → robot se mueve lateral
5. Soltar → robot se detiene

**Verificar en terminal:**
```bash
ros2 topic echo /model/tadeocar_v1/joint/front_left_steering_joint/cmd_effort
```

**Debería mostrar valores cambiantes**

**✅ PASS:** Si robot responde al joystick web
**❌ FAIL:** Si no responde → verificar WebSocket

---

#### Modo 2: Ackermann

**En la interfaz web:**
1. Click en botón **"Ackermann"**
2. Aparecen sliders de dirección y aceleración
3. Mover slider "Dirección" → solo ruedas delanteras giran
4. Mover slider "Aceleración" → robot avanza
5. Combinar ambos → robot gira como auto

**✅ PASS:** Si solo ruedas delanteras responden
**❌ FAIL:** Si todas giran → error en lógica

---

#### Modo 3: Halo (Cangrejo)

**En la interfaz web:**
1. Click en botón **"Halo"**
2. Aparecen sliders de ángulo global y velocidad
3. Mover "Ángulo global" a 90°
4. Mover "Velocidad" → robot se mueve lateral
5. Cambiar ángulo a 45° → movimiento diagonal

**✅ PASS:** Si todas las ruedas giran al mismo ángulo
**❌ FAIL:** Si ángulos son diferentes

---

#### Modo 4: Giro (Spin)

**En la interfaz web:**
1. Click en botón **"Giro"**
2. Aparece slider de velocidad de giro
3. Mover slider a la derecha (+) → gira horario
4. Mover slider a la izquierda (-) → gira antihorario
5. Robot debe girar sobre su eje

**✅ PASS:** Si robot gira en su lugar
**❌ FAIL:** Si se desplaza → ajustar cinemática

---

### 5.5 Botón STOP de Emergencia

**En cualquier modo:**
1. Poner robot en movimiento
2. Click en botón **"STOP"** (rojo grande)
3. Robot debe detenerse INMEDIATAMENTE
4. Todos los sliders vuelven a 0

**✅ PASS:** Si detiene inmediatamente
**❌ FAIL:** Si continúa moviéndose → problema crítico

---

### 5.6 Control de Velocidad Global

**Instrucciones:**
1. Mover slider "Velocidad Global" a 50%
2. Probar movimiento → debe ser mitad de velocidad
3. Mover a 100%
4. Probar movimiento → velocidad completa
5. Mover a 20%
6. Probar movimiento → muy lento

**✅ PASS:** Si velocidad cambia proporcionalmente
**❌ FAIL:** Si no afecta → verificar cálculo en JS

---

### 5.7 Tabla Resumen - Pruebas WebSocket

| Modo | Controles | Resultado Esperado |
|------|-----------|-------------------|
| Omnidireccional | Joystick virtual | Movimiento 360° |
| Ackermann | Slider dirección + aceleración | Solo ruedas delanteras |
| Halo | Slider ángulo + velocidad | Todas ruedas mismo ángulo |
| Giro | Slider giro | Rotación en lugar |
| STOP | Botón rojo | Detención inmediata |

---

## 6. PRUEBA DE SLAM

### 6.1 Preparación para SLAM

**Requisitos:**
- ✅ Xbox controller conectado (para mover robot)
- ✅ Espacio abierto en Gazebo (mundo default)
- ⏱️ Tiempo: 10-15 minutos de mapeo

---

### 6.2 Lanzar Sistema SLAM Completo

**Cerrar todos los procesos anteriores**

**Terminal 1:**
```bash
cd ~/ros2/TadeoeCar-ws
source install/setup.bash
ros2 launch tadeocar_bringup slam_bringup.launch.py
```

**Resultado esperado:**
- Gazebo se abre con robot
- RViz2 se abre (puede tardar)
- joy_node, xbox_control, fourws_kinematics activos
- slam_toolbox_node se lanza
- Mensajes: "SLAM Toolbox started"

**⏱️ Tiempo de inicio:** ~30-40 segundos

**✅ PASS:** Si todo se lanza sin errores
**❌ FAIL:** Si algún nodo falla → revisar logs

---

### 6.3 Verificar SLAM Toolbox

**Terminal 2:**
```bash
source install/setup.bash

# Verificar nodo SLAM
ros2 node list | grep slam
```

**Resultado esperado:**
```
/slam_toolbox
```

**Verificar topics SLAM:**
```bash
ros2 topic list | grep map
```

**Resultado esperado:**
```
/map
/map_metadata
```

**✅ PASS:** Si SLAM Toolbox está activo
**❌ FAIL:** Si no aparece → verificar slam_params.yaml

---

### 6.4 Configurar RViz para SLAM

**En RViz2 (debería estar abierto):**

**Si RViz está vacío o mal configurado:**

1. **Fixed Frame:**
   - En panel izquierdo arriba
   - Cambiar "Fixed Frame" a: `map`

2. **Agregar visualización del mapa:**
   - Click "Add" (abajo izquierda)
   - Seleccionar "Map"
   - Topic: `/map`
   - Click OK

3. **Agregar LIDAR:**
   - Click "Add"
   - Seleccionar "LaserScan"
   - Topic: `/scan`
   - Fixed Frame: `base_scan`
   - Color: rojo o amarillo

4. **Agregar Robot Model:**
   - Click "Add"
   - Seleccionar "RobotModel"

**Resultado esperado en RViz:**
- Mapa (gris) en el centro
- Robot (modelo 3D) visible
- Scan LIDAR (puntos rojos) alrededor del robot

**✅ PASS:** Si se ve todo
**❌ FAIL:** Si no aparece → verificar Fixed Frame y topics

---

### 6.5 Realizar Mapeo

**Usando Xbox controller:**

**Fase 1: Movimiento inicial (2 min)**
1. Mover joystick izquierdo hacia adelante
2. Avanzar ~5 metros
3. Observar en RViz: mapa empieza a aparecer
4. Líneas del mundo se dibujan en el mapa

**Fase 2: Exploración lateral (3 min)**
1. Girar 90° (mover joystick creando rotación)
2. Avanzar hacia un lado
3. Observar: mapa se expande
4. Regresar al punto inicial

**Fase 3: Completar circuito (5 min)**
1. Hacer un circuito completo del ambiente
2. Loop closing: al regresar al inicio, SLAM corrige el mapa
3. Observar: mapa se "cierra" y ajusta

**Verificar en terminal:**
```bash
ros2 topic echo /map/metadata
```

**Resultado esperado:**
```yaml
map_load_time: ...
resolution: 0.05  # 5 cm por pixel
width: 384  # Tamaño en pixels
height: 384
origin:
  position:
    x: -9.6
    y: -9.6
    z: 0.0
```

**✅ PASS:** Si mapa se construye progresivamente
**❌ FAIL:** Si mapa no aparece → verificar /scan y SLAM params

---

### 6.6 Verificar Loop Closure

**Instrucciones:**
1. Marcar mentalmente una esquina del mapa
2. Hacer circuito completo
3. Volver exactamente a esa esquina
4. Observar RViz: mapa se ajusta/corrige automáticamente
5. Mensaje en terminal: "Loop closure detected"

**✅ PASS:** Si detecta loop closure
**❌ FAIL:** Si mapa tiene discontinuidades → ajustar params

---

### 6.7 Guardar Mapa

**Método 1: Launch file**
```bash
# Terminal 3
source install/setup.bash
ros2 launch tadeocar_bringup save_map.launch.py map_name:=test_map
```

**Método 2: Comando directo**
```bash
source install/setup.bash
cd ~
ros2 run nav2_map_server map_saver_cli -f my_test_map
```

**Resultado esperado:**
- Archivo: `test_map.pgm` (imagen del mapa)
- Archivo: `test_map.yaml` (metadatos)

**Verificar archivos:**
```bash
ls -lh ~/test_map.*
```

**Resultado esperado:**
```
-rw-rw-r-- 1 axioma axioma 123K Nov  2 12:00 test_map.pgm
-rw-rw-r-- 1 axioma axioma  195 Nov  2 12:00 test_map.yaml
```

**Ver mapa guardado:**
```bash
eog ~/test_map.pgm
```

**Resultado esperado:**
- Imagen en blanco y negro
- Blanco = espacio libre
- Negro = obstáculos
- Gris = desconocido

**✅ PASS:** Si archivos se crean correctamente
**❌ FAIL:** Si no se guarda → verificar permisos

---

### 6.8 Verificar Calidad del Mapa

**Características de un buen mapa:**
- ✅ Paredes bien definidas (líneas continuas negras)
- ✅ Espacios libres blancos (sin ruido)
- ✅ Loop closure correcto (sin desplazamientos)
- ✅ Esquinas alineadas correctamente

**Características de un mal mapa:**
- ❌ Paredes fragmentadas
- ❌ Ruido (puntos grises por todo)
- ❌ Duplicación de paredes
- ❌ Desalineación en loop closure

**Si el mapa es malo:**
- Reducir velocidad de movimiento
- Ajustar `slam_params.yaml`:
  - `minimum_travel_distance: 0.3` (más frecuente)
  - `loop_match_minimum_response_coarse: 0.3` (más permisivo)

---

## 7. PRUEBA DE NAVEGACIÓN (NAV2)

### 7.1 Estado de NAV2

**⚠️ IMPORTANTE:** NAV2 NO está completamente implementado.

**Lo que existe:**
- ✅ `nav2_params.yaml` (copiado de axioma)
- ❌ NO existe `navigation_bringup.launch.py`
- ❌ Parámetros no ajustados para TadeoeCar

**Estimado para completar:** 8-12 horas

---

### 7.2 Lo que se Necesita para NAV2

**Archivos a crear:**
1. `tadeocar_bringup/launch/navigation_bringup.launch.py`
2. `tadeocar_bringup/launch/localization.launch.py`
3. Ajustar `nav2_params.yaml` para robot 4WD4WS

**Nodos necesarios:**
- Map Server (cargar mapa guardado)
- AMCL (localización)
- Controller Server (seguir trayectorias)
- Planner Server (planificar rutas)
- Behavior Server (comportamientos)
- BT Navigator (árbol de comportamiento)

---

### 7.3 Prueba Básica (Si NAV2 estuviera completo)

**NOTA: Esto NO funcionará hasta implementar NAV2**

**Hipotético:**
```bash
# Lanzar navegación
ros2 launch tadeocar_bringup navigation_bringup.launch.py map:=/path/to/map.yaml

# En RViz:
# 1. Click "2D Pose Estimate"
# 2. Click en posición inicial del robot
# 3. Click "2D Goal Pose"
# 4. Click en destino
# 5. Robot navega autónomamente
```

---

## 8. TROUBLESHOOTING GENERAL

### 8.1 Gazebo No Inicia

**Síntomas:**
- Gazebo crashea
- Pantalla negra
- Error: "Unable to create the rendering window"

**Soluciones:**
```bash
# Matar procesos anteriores
killall -9 gzserver gzclient

# Limpiar cache
rm -rf ~/.gazebo/

# Reintentar
ros2 launch tadeocar_bringup gazebo.launch.py
```

---

### 8.2 Robot No Se Mueve

**Verificar:**
```bash
# 1. Nodos activos
ros2 node list

# 2. Topics de esfuerzo
ros2 topic list | grep cmd_effort

# 3. Publicación de esfuerzos
ros2 topic echo /model/tadeocar_v1/joint/front_left_wheel_joint/cmd_effort

# 4. Servicio de Gazebo
gz service --list | grep apply_joint_effort
```

**Si no hay esfuerzos:**
- Verificar que fourws_kinematics_node está corriendo
- Verificar que gazebo_effort_bridge está corriendo

---

### 8.3 LIDAR No Publica Datos

**Verificar:**
```bash
# Topic existe
ros2 topic list | grep scan

# Información del topic
ros2 topic info /scan

# Frecuencia
ros2 topic hz /scan
```

**Si no hay datos:**
- Verificar plugin en `model.sdf`
- Revisar frame_id: debe ser `base_scan`

---

### 8.4 Xbox Controller No Detectado

**Verificar:**
```bash
# Dispositivo
ls /dev/input/js0

# Permisos
sudo chmod 666 /dev/input/js0

# Probar
jstest /dev/input/js0
```

**Si no aparece:**
- Reconectar USB
- Probar con otro cable
- Verificar con `dmesg | tail`

---

### 8.5 WebSocket No Conecta

**Verificar:**
```bash
# Servidor WebSocket corriendo
ros2 node list | grep web_control

# Puerto abierto
netstat -tulpn | grep 8765

# Logs
ros2 node info /tadeocar_web_control
```

**Si no conecta:**
- Verificar firewall
- Probar: `telnet localhost 8765`

---

## 9. CHECKLIST DE PRUEBAS

### Pruebas Básicas (30 min)

- [ ] Compilación exitosa
- [ ] Gazebo se abre
- [ ] Robot aparece en simulación
- [ ] LIDAR publica a 20 Hz
- [ ] Joint states activos
- [ ] TF tree correcto

### Pruebas de Movimiento (45 min)

- [ ] Avance hacia adelante
- [ ] Movimiento lateral
- [ ] Movimiento diagonal
- [ ] Rotación en lugar
- [ ] Modo Ackermann
- [ ] Modo Cangrejo

### Pruebas Xbox (30 min)

- [ ] Xbox conectado (/dev/input/js0)
- [ ] joy_node publica datos
- [ ] Joystick izquierdo → omnidireccional
- [ ] Joystick derecho → cangrejo
- [ ] Botones cambian modos
- [ ] Deadzone funciona

### Pruebas WebSocket (45 min)

- [ ] Interfaz web carga
- [ ] WebSocket conecta
- [ ] Modo Omnidireccional
- [ ] Modo Ackermann
- [ ] Modo Halo
- [ ] Modo Giro
- [ ] Botón STOP funciona
- [ ] Control velocidad global

### Pruebas SLAM (1 hora)

- [ ] SLAM Toolbox inicia
- [ ] RViz muestra mapa
- [ ] Mapeo progresivo
- [ ] Loop closure detectado
- [ ] Mapa se guarda
- [ ] Calidad del mapa buena

### Pruebas NAV2 (PENDIENTE)

- [ ] Map Server carga mapa
- [ ] AMCL localiza robot
- [ ] Planificación de rutas
- [ ] Navegación autónoma
- [ ] Evitación de obstáculos

---

## 10. REPORTE DE PRUEBAS

### Plantilla de Reporte

```
FECHA: ____________________
OPERADOR: _________________

SISTEMA:
[ ] Compilación OK
[ ] Gazebo OK
[ ] Sensores OK

MOVIMIENTO:
[ ] Omnidireccional OK
[ ] Ackermann OK
[ ] Cangrejo OK

CONTROL XBOX:
[ ] Conexión OK
[ ] Joystick izq OK
[ ] Joystick der OK
[ ] Botones OK

WEBSOCKET:
[ ] Conexión OK
[ ] Todos modos OK
[ ] STOP OK

SLAM:
[ ] Mapeo OK
[ ] Loop closure OK
[ ] Guardar mapa OK

OBSERVACIONES:
_________________________________
_________________________________
_________________________________

PROBLEMAS ENCONTRADOS:
_________________________________
_________________________________
_________________________________
```

---

**FIN DE LA GUÍA DE PRUEBAS**

**Última actualización:** 2025-11-02
**Versión:** 1.0
