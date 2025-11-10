# 🔄 Cinemática del Robot 4WD4WS

> **NOTA**: Este documento presenta modelos matemáticos estándar para robots 4WS. Los parámetros geométricos (L, W, r) son valores REALES de `fourws_kinematics_node.py`. Las ecuaciones cinemáticas son derivaciones matemáticas basadas en la teoría de robots móviles omnidireccionales.

## 1. Introducción

La cinemática de un robot móvil 4WD4WS describe la relación entre las velocidades de las ruedas (ángulos de dirección y velocidades angulares) y la velocidad del robot como cuerpo rígido. Este documento presenta el modelo matemático completo para los tres modos de operación implementados.

---

## 2. Geometría del Robot

### 2.1 Posición de las Ruedas

Sea el centro geométrico del robot el origen del sistema de coordenadas $\{R\}$. Las posiciones de las ruedas en el plano $xy$ son:

$$
\mathbf{p}_{FL} = \begin{bmatrix} \frac{L}{2} \\ \frac{W}{2} \end{bmatrix}, \quad
\mathbf{p}_{FR} = \begin{bmatrix} \frac{L}{2} \\ -\frac{W}{2} \end{bmatrix}
$$

$$
\mathbf{p}_{RL} = \begin{bmatrix} -\frac{L}{2} \\ \frac{W}{2} \end{bmatrix}, \quad
\mathbf{p}_{RR} = \begin{bmatrix} -\frac{L}{2} \\ -\frac{W}{2} \end{bmatrix}
$$

Donde:
- $L = 1.058$ m (wheel_base: distancia entre ejes delantero y trasero)
- $W = 0.55$ m (track_width: distancia entre ruedas izquierda y derecha)

### 2.2 Sistema de Coordenadas

```
        y (izquierda)
        ↑
        |     FL •━━━━━• FR
        |        |     |
        |        | [R] |  (robot frame)
        |        |     |
        |     RL •━━━━━• RR
        |
        └─────────────────→ x (adelante)

                θ (yaw, antihorario positivo)
```

---

## 3. Modelo Cinemático General

### 3.1 Velocidad del Robot

El vector de velocidad del robot en su propio frame $\{R\}$ es:

$$
\mathbf{v}_R = \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}
$$

Donde:
- $v_x$: Velocidad lineal en el eje $x$ (adelante/atrás)
- $v_y$: Velocidad lineal en el eje $y$ (izquierda/derecha)
- $\omega$: Velocidad angular alrededor del eje $z$ (rotación)

### 3.2 Velocidad de Cada Rueda

La velocidad del punto de contacto de la rueda $i$ con respecto al centro del robot es:

$$
\mathbf{v}_{w_i} = \begin{bmatrix} v_x \\ v_y \end{bmatrix} + \omega \begin{bmatrix} -p_{y_i} \\ p_{x_i} \end{bmatrix}
$$

Donde $\mathbf{p}_i = [p_{x_i}, p_{y_i}]^T$ es la posición de la rueda $i$ definida en 2.1.

### 3.3 Restricciones de Rodadura

Para una rueda con ángulo de dirección $\alpha_i$, el vector unitario en la dirección de rodadura es:

$$
\hat{\mathbf{d}}_i = \begin{bmatrix} \cos\alpha_i \\ \sin\alpha_i \end{bmatrix}
$$

La velocidad de rodadura de la rueda es:

$$
v_{roll_i} = \mathbf{v}_{w_i}^T \hat{\mathbf{d}}_i = v_x \cos\alpha_i + v_y \sin\alpha_i + \omega(-p_{y_i}\cos\alpha_i + p_{x_i}\sin\alpha_i)
$$

La velocidad angular de la rueda $\dot{\phi}_i$ está relacionada con la velocidad de rodadura:

$$
\dot{\phi}_i = \frac{v_{roll_i}}{r}
$$

Donde $r = 0.1$ m es el radio de la rueda.

---

## 4. Cinemática Inversa por Modo

### 4.1 Modo Omnidireccional

**Principio**: Todas las ruedas apuntan en la dirección del vector de velocidad instantánea, permitiendo movilidad omnidireccional completa.

#### 4.1.1 Ángulos de Dirección

Para movimiento translacional puro $(v_x, v_y, \omega=0)$:

$$
\alpha_i = \text{atan2}(v_y, v_x), \quad \forall i \in \{FL, FR, RL, RR\}
$$

Para movimiento con rotación $(\omega \neq 0)$:

$$
\alpha_i = \text{atan2}\left(v_y - \omega \cdot p_{x_i}, v_x + \omega \cdot p_{y_i}\right)
$$

**Caso especial**: Si $v_x = v_y = 0$ y $\omega \neq 0$ (rotación pura), los ángulos son:

$$
\begin{aligned}
\alpha_{FL} &= \text{atan2}\left(-\frac{L}{2}, \frac{W}{2}\right) = -\text{atan2}(L, W) \\
\alpha_{FR} &= \text{atan2}\left(-\frac{L}{2}, -\frac{W}{2}\right) = -\pi + \text{atan2}(L, W) \\
\alpha_{RL} &= \text{atan2}\left(\frac{L}{2}, \frac{W}{2}\right) = \text{atan2}(L, W) \\
\alpha_{RR} &= \text{atan2}\left(\frac{L}{2}, -\frac{W}{2}\right) = \pi - \text{atan2}(L, W)
\end{aligned}
$$

#### 4.1.2 Velocidades Angulares de Ruedas

$$
\dot{\phi}_i = \frac{1}{r} \sqrt{(v_x + \omega \cdot p_{y_i})^2 + (v_y - \omega \cdot p_{x_i})^2}
$$

Con signo determinado por:

$$
\text{sign}(\dot{\phi}_i) = \text{sign}\left(\cos\alpha_i (v_x + \omega \cdot p_{y_i}) + \sin\alpha_i (v_y - \omega \cdot p_{x_i})\right)
$$

---

### 4.2 Modo Ackermann

**Principio**: Geometría de dirección tipo automóvil con centro instantáneo de rotación (ICR) compartido.

#### 4.2.1 Centro Instantáneo de Rotación

Para velocidades $(v_x, \omega)$ con $\omega \neq 0$:

$$
R = \frac{v_x}{\omega}
$$

Donde $R$ es la distancia desde el centro del robot al ICR (positivo: giro izquierda, negativo: giro derecha).

#### 4.2.2 Ángulos de Dirección

**Ruedas delanteras** (Ackermann steering):

$$
\begin{aligned}
\alpha_{FL} &= \text{atan2}\left(L, R - \frac{W}{2}\right) = \text{atan}\left(\frac{L \cdot \omega}{v_x - \frac{W}{2}\omega}\right) \\
\alpha_{FR} &= \text{atan2}\left(L, R + \frac{W}{2}\right) = \text{atan}\left(\frac{L \cdot \omega}{v_x + \frac{W}{2}\omega}\right)
\end{aligned}
$$

**Ruedas traseras** (paralelas al eje del robot):

$$
\alpha_{RL} = \alpha_{RR} = 0
$$

**Caso lineal** ($\omega = 0$):

$$
\alpha_{FL} = \alpha_{FR} = \alpha_{RL} = \alpha_{RR} = 0
$$

#### 4.2.3 Velocidades Angulares de Ruedas

Las velocidades son proporcionales a la distancia desde cada rueda al ICR:

$$
\begin{aligned}
\dot{\phi}_{FL} &= \frac{v_x}{r} \sqrt{1 + \left(\frac{L\omega}{v_x - \frac{W}{2}\omega}\right)^2} \\
\dot{\phi}_{FR} &= \frac{v_x}{r} \sqrt{1 + \left(\frac{L\omega}{v_x + \frac{W}{2}\omega}\right)^2} \\
\dot{\phi}_{RL} &= \frac{v_x - \frac{W}{2}\omega}{r} \\
\dot{\phi}_{RR} &= \frac{v_x + \frac{W}{2}\omega}{r}
\end{aligned}
$$

---

### 4.3 Modo Crab

**Principio**: Movimiento lateral con todas las ruedas al mismo ángulo (desplazamiento perpendicular al chasis).

#### 4.3.1 Ángulos de Dirección

Todas las ruedas se orientan en la misma dirección:

$$
\alpha_{FL} = \alpha_{FR} = \alpha_{RL} = \alpha_{RR} = \text{atan2}(v_y, v_x)
$$

Para movimiento lateral puro ($v_x = 0$):

$$
\alpha_i = \begin{cases}
+\frac{\pi}{2} & \text{si } v_y > 0 \text{ (izquierda)} \\
-\frac{\pi}{2} & \text{si } v_y < 0 \text{ (derecha)}
\end{cases}
$$

#### 4.3.2 Velocidades Angulares de Ruedas

Velocidad uniforme para todas las ruedas:

$$
\dot{\phi}_{FL} = \dot{\phi}_{FR} = \dot{\phi}_{RL} = \dot{\phi}_{RR} = \frac{\sqrt{v_x^2 + v_y^2}}{r}
$$

---

## 5. Cinemática Directa

La cinemática directa calcula la velocidad del robot $\mathbf{v}_R = [v_x, v_y, \omega]^T$ a partir de los ángulos de dirección $\alpha_i$ y velocidades angulares $\dot{\phi}_i$.

### 5.1 Modelo General

Para cada rueda, la velocidad de rodadura es:

$$
v_{roll_i} = r \cdot \dot{\phi}_i
$$

La velocidad del punto de contacto:

$$
\mathbf{v}_{w_i} = v_{roll_i} \begin{bmatrix} \cos\alpha_i \\ \sin\alpha_i \end{bmatrix}
$$

Sistema de ecuaciones (4 ruedas × 2 componentes = 8 ecuaciones):

$$
\begin{bmatrix} v_x \\ v_y \end{bmatrix} + \omega \begin{bmatrix} -p_{y_i} \\ p_{x_i} \end{bmatrix} = r\dot{\phi}_i \begin{bmatrix} \cos\alpha_i \\ \sin\alpha_i \end{bmatrix}
$$

### 5.2 Solución por Mínimos Cuadrados

Dado que el sistema está sobredeterminado (8 ecuaciones, 3 incógnitas), se usa:

$$
\mathbf{A} \mathbf{v}_R = \mathbf{b}
$$

Donde:

$$
\mathbf{A} = \begin{bmatrix}
1 & 0 & -p_{y_{FL}} \\
0 & 1 & p_{x_{FL}} \\
1 & 0 & -p_{y_{FR}} \\
0 & 1 & p_{x_{FR}} \\
1 & 0 & -p_{y_{RL}} \\
0 & 1 & p_{x_{RL}} \\
1 & 0 & -p_{y_{RR}} \\
0 & 1 & p_{x_{RR}}
\end{bmatrix}, \quad
\mathbf{b} = \begin{bmatrix}
r\dot{\phi}_{FL}\cos\alpha_{FL} \\
r\dot{\phi}_{FL}\sin\alpha_{FL} \\
r\dot{\phi}_{FR}\cos\alpha_{FR} \\
r\dot{\phi}_{FR}\sin\alpha_{FR} \\
r\dot{\phi}_{RL}\cos\alpha_{RL} \\
r\dot{\phi}_{RL}\sin\alpha_{RL} \\
r\dot{\phi}_{RR}\cos\alpha_{RR} \\
r\dot{\phi}_{RR}\sin\alpha_{RR}
\end{bmatrix}
$$

Solución:

$$
\mathbf{v}_R = (\mathbf{A}^T\mathbf{A})^{-1}\mathbf{A}^T\mathbf{b}
$$

---

## 6. Transformada de Pose

### 6.1 Integración de Velocidad

La pose del robot en el frame mundial $\{W\}$ evoluciona según:

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}_W =
\begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}_R
$$

### 6.2 Integración Numérica (Euler)

Con período de muestreo $\Delta t$:

$$
\begin{aligned}
x_{k+1} &= x_k + (v_x \cos\theta_k - v_y \sin\theta_k) \Delta t \\
y_{k+1} &= y_k + (v_x \sin\theta_k + v_y \cos\theta_k) \Delta t \\
\theta_{k+1} &= \theta_k + \omega \Delta t
\end{aligned}
$$

---

## 7. Restricciones y Límites

### 7.1 Límites Físicos

$$
\begin{aligned}
|\alpha_i| &\leq \alpha_{max} = 1.57 \text{ rad} \quad (90°) \\
|v_x| + |v_y| &\leq v_{max} = 2.0 \text{ m/s} \\
|\omega| &\leq \omega_{max} = 1.0 \text{ rad/s} \\
|\dot{\phi}_i| &\leq \frac{v_{max}}{r} = 20.0 \text{ rad/s}
\end{aligned}
$$

### 7.2 Saturación de Comandos

Implementación de límites en `fourws_kinematics_node.py:85-92`:

$$
v_x' = \text{clip}(v_x, -v_{max}, v_{max})
$$

$$
v_y' = \text{clip}(v_y, -v_{max}, v_{max})
$$

$$
\omega' = \text{clip}(\omega, -\omega_{max}, \omega_{max})
$$

---

## 8. Implementación Computacional

### 8.1 Pseudocódigo: Modo Omnidireccional

```python
def compute_omnidirectional(vx, vy, omega):
    for i in [FL, FR, RL, RR]:
        # Posición de la rueda
        px, py = wheel_positions[i]

        # Velocidad del punto de contacto
        vwx = vx + omega * (-py)
        vwy = vy + omega * px

        # Ángulo de dirección
        alpha[i] = atan2(vwy, vwx)

        # Velocidad angular de la rueda
        v_roll = sqrt(vwx^2 + vwy^2)
        phi_dot[i] = v_roll / wheel_radius

    return alpha, phi_dot
```

### 8.2 Pseudocódigo: Modo Ackermann

```python
def compute_ackermann(vx, omega):
    if abs(omega) < epsilon:
        # Movimiento recto
        alpha = [0, 0, 0, 0]
        phi_dot = [vx/r] * 4
    else:
        # Cálculo del radio de giro
        R = vx / omega

        # Ángulos delanteros (Ackermann)
        alpha[FL] = atan(L / (R - W/2))
        alpha[FR] = atan(L / (R + W/2))

        # Ángulos traseros (rectos)
        alpha[RL] = 0
        alpha[RR] = 0

        # Velocidades proporcionales a distancia del ICR
        phi_dot[FL] = compute_wheel_speed(FL, vx, omega, alpha[FL])
        phi_dot[FR] = compute_wheel_speed(FR, vx, omega, alpha[FR])
        phi_dot[RL] = (vx - W/2 * omega) / r
        phi_dot[RR] = (vx + W/2 * omega) / r

    return alpha, phi_dot
```

---

## 9. Referencias

**Implementación**: `src/tadeocar_control/tadeocar_control/fourws_kinematics_node.py`

**Ecuaciones clave**:
- Líneas 95-110: Modo Omnidireccional
- Líneas 115-145: Modo Ackermann
- Líneas 150-165: Modo Crab

**Parámetros**: Ver `documentacion/modelo-matematico/parametros.md`

---

**Autor**: Tadeo Robotics Group
**Fecha**: 2025
**Versión**: 1.0.0
