# Modelo Matematico - Robot Tadeo eCar 4WD4WS

## 1. Parametros Fisicos

| Parametro | Simbolo | Valor | Unidad |
|-----------|---------|-------|--------|
| Radio de rueda | r | 0.1 | m |
| Distancia entre ejes (wheelbase) | L | 1.058 | m |
| Ancho de via (track width) | W | 0.55 | m |
| Masa del chasis base | - | 50.0 | kg |
| Masa del chasis superior | - | 10.0 | kg |
| Masa por rueda+suspension | - | 12.0 | kg |
| Rango de giro del servo | - | 270 | grados |
| Angulo maximo de steering | alpha_max | +/-135 | grados (2.356 rad) |
| Velocidad lineal maxima | v_max | 2.0 | m/s |
| Velocidad angular maxima | w_max | 1.0 | rad/s |

### Posiciones de las ruedas respecto al centro del robot

| Rueda | x (m) | y (m) |
|-------|-------|-------|
| Front Left (FL) | +L/2 = +0.529 | +W/2 = +0.275 |
| Front Right (FR) | +L/2 = +0.529 | -W/2 = -0.275 |
| Rear Left (RL) | -L/2 = -0.529 | +W/2 = +0.275 |
| Rear Right (RR) | -L/2 = -0.529 | -W/2 = -0.275 |

---

## 2. Sistema de Control

### Steering: Control por posicion

Los 4 joints de steering utilizan `JointPositionController` de Gz Sim, que replica el
comportamiento de servomotores industriales de 270 grados controlados por posicion.

El nodo de cinematica publica directamente el angulo objetivo (radianes) en cada topic:
```
/model/tadeocar/joint/{wheel}_steering_joint/cmd_pos  -> Float64 (radianes)
```

El PID interno del plugin de Gazebo se encarga de alcanzar la posicion:
- P = 50.0
- I = 0.0
- D = 10.0

### Ruedas: Control por velocidad

Los 4 joints de traccion utilizan `JointController` con comandos de velocidad angular:
```
/model/tadeocar/joint/{wheel}_wheel_joint/cmd_vel  -> Float64 (rad/s)
```

---

## 3. Cinematica Inversa

Dado un comando de velocidad (vx, vy, wz) en el frame del robot, se calculan los angulos
de steering (alpha_i) y las velocidades angulares de rueda (phi_dot_i) para cada rueda i.

### 3.1 Modo Omnidireccional

Cada rueda tiene un angulo de steering independiente calculado a partir de su velocidad
instantanea. Para la rueda i con posicion (px_i, py_i):

Velocidad instantanea de la rueda:
```
vw_x_i = vx - wz * py_i
vw_y_i = vy + wz * px_i
```

Angulo de steering:
```
alpha_i = atan2(vw_y_i, vw_x_i)
```

Si |alpha_i| > pi/2, se normaliza al rango [-pi/2, pi/2] invirtiendo la direccion de
la rueda (la rueda gira en sentido contrario para evitar girar el servo mas de 90 grados
en modo omnidireccional).

Velocidad angular de la rueda:
```
phi_dot_i = sqrt(vw_x_i^2 + vw_y_i^2) / r * direction
```

Donde `direction` es +1 o -1 segun la normalizacion del angulo.

### 3.2 Modo Ackermann

Solo las ruedas delanteras giran. Las traseras se mantienen a 0 grados.
Dado un radio de giro R = vx / wz:

```
alpha_FL = atan2(L, R - W/2)
alpha_FR = atan2(L, R + W/2)
alpha_RL = 0
alpha_RR = 0
```

Velocidades diferenciales:
```
vel_diff = wz * (W/2) / r
phi_dot_FL = vx/r - vel_diff
phi_dot_FR = vx/r + vel_diff
phi_dot_RL = vx/r - vel_diff
phi_dot_RR = vx/r + vel_diff
```

### 3.3 Modo Crab

Todas las ruedas apuntan en la misma direccion para movimiento lateral.
Las ruedas delanteras y traseras tienen angulos opuestos:

```
lateral_angle = atan2(|vy|, |vx|)
alpha_front = sign(vy) * lateral_angle
alpha_rear = -alpha_front
```

Todas las ruedas giran a la misma velocidad:
```
phi_dot = sqrt(vx^2 + vy^2) / r
```

---

## 4. Transformada de Velocidad (Robot -> Mundo)

Para transformar velocidades del frame del robot {R} al frame mundial {W}:

```
[x_dot]       [cos(theta)  -sin(theta)  0] [vx]
[y_dot]     = [sin(theta)   cos(theta)  0] [vy]
[theta_dot]   [0            0           1] [wz]
```

Donde theta es la orientacion del robot en el frame mundial.

---

## 5. Script de Analisis

El archivo `src/tadeocar_control/tadeocar_control/cinematica.py` genera graficas de
contorno que muestran las velocidades angulares de las ruedas (RPM) y los angulos de
steering (grados) en funcion de la velocidad lineal y angular deseada.

Ejecutar:
```bash
python3 src/tadeocar_control/tadeocar_control/cinematica.py
```

---

## 6. Referencias

- Codigo fuente: `src/tadeocar_control/tadeocar_control/fourws_kinematics_node.py`
- Modelo SDF: `src/tadeocar_gazebo/models/tadeocar_v1/model.sdf`
- JointPositionController: https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1JointPositionController.html
