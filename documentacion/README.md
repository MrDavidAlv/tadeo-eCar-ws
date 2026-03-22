# Documentacion Tecnica - Robot Tadeo eCar 4WD4WS

## Control basado en CMD_POS

El sistema de control del robot usa **control por posicion (cmd_pos)** para los servos de steering
y **control por velocidad (cmd_vel)** para las ruedas de traccion. El nodo `fourws_kinematics_node`
es el unico que publica directamente a los joints de Gazebo:

- Steering: `/model/tadeocar/joint/{wheel}_steering_joint/cmd_pos` (Float64, radianes)
- Traccion: `/model/tadeocar/joint/{wheel}_wheel_joint/cmd_vel` (Float64, rad/s)

Todos los demas nodos (web_control, nav2, teleop) publican en `/cmd_vel` (Twist) y `/robot_mode` (String),
y `fourws_kinematics_node` se encarga de la conversion cinematica a comandos de joints.

## Contenido

### [Modelo Matematico](./modelo-matematico.md)

Documentacion del modelo cinematico y de control del robot 4WD4WS:

- Parametros fisicos del robot (dimensiones, masas, limites)
- Sistema de control: steering por posicion (cmd_pos), ruedas por velocidad (cmd_vel)
- Cinematica inversa: modos omnidireccional, Ackermann y crab
- Transformada de velocidad robot-mundo
- Script de analisis y visualizacion (`cinematica.py`)

### Diagrama Visual

El archivo [modelo-4ws.excalidraw](./modelo-4ws.excalidraw) contiene la representacion
visual del robot con sistemas de coordenadas y vectores de velocidad.
Abrir con [Excalidraw](https://excalidraw.com).

---

## Convenciones

### Notacion

- Vectores: minusculas en negrita
- Matrices: mayusculas en negrita
- Escalares: minusculas italicas (vx, vy, wz)

### Frames de referencia

- {W}: Frame mundial (map/odom)
- {R}: Frame del robot (base_link)
- {Wi}: Frame de la rueda i

### Indices de ruedas

- FL: Front Left
- FR: Front Right
- RL: Rear Left
- RR: Rear Right

---

## Referencias Cruzadas

| Documento | Implementacion |
|-----------|----------------|
| [modelo-matematico.md](./modelo-matematico.md) | `fourws_kinematics_node.py` |
| Parametros fisicos | `model.sdf` (tadeocar_gazebo) |
| Script de analisis | `cinematica.py` (documentacion/) |

---

Semillero Robotica - Universidad de Bogota Jorge Tadeo Lozano
