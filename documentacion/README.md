# Documentacion Tecnica - Robot Tadeo eCar 4WD4WS

## Contenido

### [Modelo Matematico](./modelo-matematico.md)

Documentacion del modelo cinematico y de control del robot 4WD4WS:

- Parametros fisicos del robot (dimensiones, masas, limites)
- Sistema de control: steering por posicion, ruedas por velocidad
- Cinematica inversa: modos omnidireccional, Ackermann y crab
- Transformada de velocidad robot-mundo
- Script de analisis y visualizacion

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
| Script de analisis | `cinematica.py` (tadeocar_control) |

---

Semillero Robotica - Universidad de Bogota Jorge Tadeo Lozano
