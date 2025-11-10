# 📏 Parámetros Físicos del Robot 4WD4WS

> **NOTA**: Este documento contiene valores REALES extraídos directamente del código fuente del robot. Los parámetros marcados con ✅ provienen de archivos de configuración (`model.sdf`, `fourws_kinematics_node.py`, `tadeocar_tf.urdf`). Los valores derivados o calculados están claramente identificados.

## 1. Geometría del Chasis

### 1.1 Dimensiones Principales

| Parámetro | Símbolo | Valor | Fuente | Descripción |
|-----------|---------|-------|--------|-------------|
| Distancia entre ejes | $L$ | 1.058 m | ✅ `fourws_kinematics_node.py:35` | Separación entre ejes delantero y trasero |
| Ancho de vía | $W$ | 0.55 m | ✅ `fourws_kinematics_node.py:36` | Separación entre ruedas izquierda y derecha |
| Radio de rueda | $r$ | 0.1 m | ✅ `fourws_kinematics_node.py:34` | Radio efectivo de rodadura |
| Diámetro de rueda | $d$ | 0.2 m | ✅ Derivado: $d = 2r$ | Diámetro total de la rueda |
| Altura del chasis | $h$ | 0.45 m | ✅ `model.sdf:48` (pose chasis_link) | Altura del chasis superior |

### 1.2 Posiciones de Ruedas en Frame {R}

$$
\mathbf{p}_{FL} = \begin{bmatrix} 0.478 \\ 0.275 \\ 0.1 \end{bmatrix} \text{ m}, \quad
\mathbf{p}_{FR} = \begin{bmatrix} 0.478 \\ -0.275 \\ 0.1 \end{bmatrix} \text{ m}
$$

$$
\mathbf{p}_{RL} = \begin{bmatrix} -0.58 \\ 0.275 \\ 0.1 \end{bmatrix} \text{ m}, \quad
\mathbf{p}_{RR} = \begin{bmatrix} -0.58 \\ -0.275 \\ 0.1 \end{bmatrix} \text{ m}
$$

**Nota**: Las coordenadas en $x$ no son simétricas debido a la distribución de peso del chasis.

### 1.3 Relaciones Geométricas

Distancia del centro a cada rueda (radio de rotación):

$$
d_i = \sqrt{p_{x_i}^2 + p_{y_i}^2}
$$

Para FL:

$$
d_{FL} = \sqrt{0.478^2 + 0.275^2} = \sqrt{0.304} = 0.551 \text{ m}
$$

Diagonal del robot:

$$
d_{diag} = \sqrt{L^2 + W^2} = \sqrt{1.058^2 + 0.55^2} = 1.194 \text{ m}
$$

---

## 2. Propiedades Inerciales

### 2.1 Masas de Componentes

| Componente | Cantidad | Masa Unitaria | Masa Total |
|------------|----------|---------------|------------|
| Chasis base | 1 | 50.0 kg | 50.0 kg |
| Chasis superior | 1 | 10.0 kg | 10.0 kg |
| Suspensiones | 4 | 6.0 kg | 24.0 kg |
| Ruedas | 4 | 6.0 kg | 24.0 kg |
| **Total** | - | - | **108.0 kg** |

**Fuente**: `model.sdf:65, 125, 190, 280, ...`

### 2.2 Tensor de Inercia del Chasis

**Chasis base** (✅ Valores REALES de `model.sdf:10-18`):

$$
\mathbf{I}_{base} = \begin{bmatrix}
3.042 & 0 & 0 \\
0 & 6.375 & 0 \\
0 & 0 & 8.667
\end{bmatrix} \text{ kg·m}^2
$$

```xml
<link name="base_link">
  <mass>50.0</mass>
  <inertia>
    <ixx>3.042</ixx>
    <iyy>6.375</iyy>
    <izz>8.667</izz>
  </inertia>
</link>
```

**Chasis superior** (✅ Valores REALES de `model.sdf:51-61`):

$$
\mathbf{I}_{chasis} = \begin{bmatrix}
0.433 & 0 & 0 \\
0 & 0.667 & 0 \\
0 & 0 & 0.833
\end{bmatrix} \text{ kg·m}^2
$$

```xml
<link name="chasis_link">
  <mass>10.0</mass>
  <inertia>
    <ixx>0.433</ixx>
    <iyy>0.667</iyy>
    <izz>0.833</izz>
  </inertia>
</link>
```

### 2.3 Inercia de Suspensiones

**Steering links** (✅ Valores REALES de `model.sdf:98-107`):

$$
\mathbf{I}_{steering} = \begin{bmatrix}
0.05 & 0 & 0 \\
0 & 0.05 & 0 \\
0 & 0 & 0.05
\end{bmatrix} \text{ kg·m}^2, \quad m_{steering} = 6.0 \text{ kg}
$$

### 2.4 Inercia de Ruedas

**Wheel links** (✅ Valores REALES de `model.sdf:180-189`):

$$
\mathbf{I}_{wheel} = \begin{bmatrix}
0.0364 & 0 & 0 \\
0 & 0.06 & 0 \\
0 & 0 & 0.0364
\end{bmatrix} \text{ kg·m}^2, \quad m_{wheel} = 6.0 \text{ kg}
$$

```xml
<link name="front_left_wheel_link">
  <mass>6.0</mass>
  <inertia>
    <ixx>0.0364</ixx>
    <iyy>0.06</iyy>
    <izz>0.0364</izz>
  </inertia>
</link>
```

---

## 3. Parámetros Cinemáticos

### 3.1 Velocidades Límite

| Parámetro | Símbolo | Valor | Unidad |
|-----------|---------|-------|--------|
| Velocidad lineal máxima | $v_{max}$ | 2.0 | m/s |
| Velocidad angular máxima | $\omega_{max}$ | 1.0 | rad/s |
| Velocidad de rueda máxima | $\dot{\phi}_{max}$ | 20.0 | rad/s |
| Velocidad de dirección máxima | $\dot{\alpha}_{max}$ | 1.0 | rad/s |

**Fuente**: `fourws_kinematics_node.py:40-45`

### 3.2 Ángulos Límite

| Parámetro | Símbolo | Valor | Equivalente |
|-----------|---------|-------|-------------|
| Ángulo de dirección máximo | $\alpha_{max}$ | ±1.57 rad | ±90° |
| Rango de dirección total | $\Delta\alpha$ | 3.14 rad | 180° |
| Límite físico (SDF) | - | ±2.356 rad | ±135° |

**Nota**: El nodo cinemático usa límite software de ±90°, aunque el hardware (simulado) permite ±135°.

### 3.3 Radio de Giro Mínimo

Para modo Ackermann con $\alpha_{max} = 90°$:

$$
R_{min} = \frac{L}{\tan(\alpha_{max})} = \frac{1.058}{\tan(90°)} \rightarrow 0 \text{ m (rotación en el lugar)}
$$

En práctica, con $\alpha \approx 60°$:

$$
R_{min} = \frac{1.058}{\tan(60°)} = \frac{1.058}{1.732} = 0.611 \text{ m}
$$

---

## 4. Parámetros Dinámicos

### 4.1 Fricción Rueda-Suelo

**Coeficientes de Coulomb** (`model.sdf:300-305`):

| Coeficiente | Símbolo | Valor | Descripción |
|-------------|---------|-------|-------------|
| Fricción direccional | $\mu_1$ | 2.5 | En dirección de rodadura |
| Fricción lateral | $\mu_2$ | 2.5 | Perpendicular a rodadura |

Fuerza de fricción máxima por rueda:

$$
F_{friction,max} = \mu \cdot F_N = 2.5 \times \frac{mg}{4} = 2.5 \times \frac{108 \times 9.81}{4} = 663.15 \text{ N}
$$

### 4.2 Amortiguamiento de Juntas

**Juntas de dirección** (`model.sdf:245`):

$$
b_{steering} = 5.0 \text{ N·m·s/rad}
$$

**Juntas de ruedas** (`model.sdf:320`):

$$
b_{wheel} = 0.01 \text{ N·m·s/rad}
$$

### 4.3 Límites de Torque

| Junta | Torque Máximo | Uso |
|-------|---------------|-----|
| Steering | 100 N·m | Cambiar dirección |
| Wheel | 200 N·m | Acelerar/frenar robot |

**Aceleración máxima del robot**:

Asumiendo tracción total y 4 ruedas aplicando torque:

$$
F_{total} = 4 \times \frac{\tau_{wheel,max}}{r} = 4 \times \frac{200}{0.1} = 8000 \text{ N}
$$

$$
a_{max} = \frac{F_{total}}{m} = \frac{8000}{108} = 74.07 \text{ m/s}^2
$$

**En práctica**, limitado por fricción:

$$
a_{max,real} = \mu \cdot g = 2.5 \times 9.81 = 24.525 \text{ m/s}^2
$$

---

## 5. Sensor LiDAR

### 5.1 Especificaciones

| Parámetro | Valor |
|-----------|-------|
| Tipo | 2D planar laser scanner |
| Posición en robot | $(0.55, 0, 0.43)$ m |
| Número de muestras | 320 |
| Rango angular | ±160° (-2.79 a +2.79 rad) |
| Resolución angular | $\frac{320°}{320} = 1°$ |
| Rango mínimo | 0.36 m |
| Rango máximo | 3.5 m |
| Frecuencia de escaneo | 20 Hz |

**Fuente**: `model.sdf:748-780`

### 5.2 Transformada del LiDAR

Transformada de `base_link` → `base_scan`:

$$
\mathbf{T}_{scan}^{base} = \begin{bmatrix}
1 & 0 & 0 & 0.55 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.43 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## 6. Parámetros de Control

### 6.1 Ganancias PID

**Dirección**:

$$
K_{p,steering} = 50.0, \quad K_{d,steering} = 5.0
$$

**Ruedas** (configurado en URDF):

$$
K_{p,wheel} = 10.0, \quad K_{i,wheel} = 0.0, \quad K_{d,wheel} = 1.0
$$

**Fuente**: `tadeocar_tf.urdf:35-40`, `fourws_kinematics_node.py:48-49`

### 6.2 Frecuencias de Operación

| Sistema | Frecuencia | Período |
|---------|------------|---------|
| ros2_control update | 100 Hz | 10 ms |
| Nodo cinemático | 50 Hz | 20 ms |
| LiDAR | 20 Hz | 50 ms |
| Nav2 controller | 20 Hz | 50 ms |

---

## 7. Limitaciones Operacionales

### 7.1 Aceleración

Basada en fricción disponible:

$$
a_{x,max} = a_{y,max} = \mu \cdot g = 2.5 \times 9.81 = 24.5 \text{ m/s}^2
$$

### 7.2 Deceleración (Frenado)

Distancia de frenado desde $v_{max} = 2$ m/s:

$$
d_{brake} = \frac{v^2}{2a_{max}} = \frac{2^2}{2 \times 24.5} = 0.082 \text{ m}
$$

### 7.3 Capacidad de Escalada

Ángulo máximo de rampa (limitado por fricción):

$$
\theta_{max} = \arctan(\mu) = \arctan(2.5) = 68.2°
$$

**En práctica**, limitado por geometría del chasis (~20° antes de tocar suelo).

---

## 8. Consumo Energético (Estimado)

### 8.1 Potencia de Motores

Potencia por motor de rueda a velocidad máxima:

$$
P_{motor} = \tau \cdot \omega = 200 \times 20 = 4000 \text{ W}
$$

Potencia total (4 motores):

$$
P_{total} = 4 \times 4000 = 16000 \text{ W} = 16 \text{ kW}
$$

### 8.2 Escenario Típico

Navegación a $v = 0.5$ m/s en superficie plana:

$$
\omega_{wheel} = \frac{v}{r} = \frac{0.5}{0.1} = 5 \text{ rad/s}
$$

Torque requerido (venciendo fricción):

$$
\tau_{req} = F_{friction} \times r = (b_w \omega + F_{rolling}) \times r \approx 50 \text{ N·m}
$$

Potencia:

$$
P_{typical} = 4 \times (50 \times 5) = 1000 \text{ W} = 1 \text{ kW}
$$

---

## 9. Tabla Resumen de Parámetros

### 9.1 Parámetros Geométricos

```python
PARAMS_GEOMETRY = {
    'wheel_base': 1.058,      # m
    'track_width': 0.55,      # m
    'wheel_radius': 0.1,      # m
    'chassis_height': 0.5,    # m
    'total_mass': 108.0       # kg
}
```

### 9.2 Parámetros Cinemáticos

```python
PARAMS_KINEMATICS = {
    'max_linear_velocity': 2.0,      # m/s
    'max_angular_velocity': 1.0,     # rad/s
    'max_steering_angle': 1.57,      # rad (90°)
    'max_steering_velocity': 1.0,    # rad/s
    'max_wheel_velocity': 20.0       # rad/s
}
```

### 9.3 Parámetros de Control

```python
PARAMS_CONTROL = {
    'kp_steering': 50.0,
    'kd_steering': 5.0,
    'kp_wheel': 10.0,
    'ki_wheel': 0.0,
    'kd_wheel': 1.0,
    'control_frequency': 100.0       # Hz
}
```

---

## 10. Referencias Cruzadas

- **Cinemática**: [cinematica.md](./cinematica.md) utiliza estos parámetros en ecuaciones
- **Control**: [control.md](./control.md) utiliza ganancias y frecuencias
- **Implementación**: Valores definidos en `fourws_kinematics_node.py:35-55`
- **Simulación**: Valores físicos en `model.sdf`

---

**Autor**: Tadeo Robotics Group
**Fecha**: 2025
**Versión**: 1.0.0
