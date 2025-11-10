# 📐 Modelo Matemático del Robot 4WD4WS

## Descripción General

Este documento presenta el modelo matemático completo del robot autónomo Tadeo eCar, una plataforma móvil omnidireccional con configuración **Four-Wheel Drive, Four-Wheel Steering** (4WD4WS). El modelo abarca cinemática directa e inversa, control PID y parámetros físicos del sistema.

---

## 📑 Contenido

1. **[Cinemática del Robot](./cinematica.md)**
   - Cinemática directa
   - Cinemática inversa
   - Modos de operación (Omnidireccional, Ackermann, Crab)
   - Transformadas de coordenadas

2. **[Sistema de Control](./control.md)**
   - Control PID de dirección
   - Control PID de velocidad
   - Arquitectura ros2_control

3. **[Parámetros Físicos](./parametros.md)**
   - Geometría del robot
   - Características dinámicas
   - Limitaciones operacionales

4. **[Diagrama Excalidraw](./modelo-4ws.excalidraw)**
   - Representación visual del modelo
   - Sistemas de coordenadas
   - Vectores cinemáticos

---

## Notación Matemática

### Sistemas de Coordenadas

| Símbolo | Descripción | Frame |
|---------|-------------|-------|
| $\\{W\\}$ | Sistema de coordenadas mundial | `map` / `odom` |
| $\\{R\\}$ | Sistema de coordenadas del robot | `base_link` |
| $\\{W_i\\}$ | Sistema de coordenadas de la rueda $i$ | `wheel_link` |

### Variables de Estado

| Variable | Descripción | Unidad |
|----------|-------------|--------|
| $q = [x, y, \\theta]^T$ | Pose del robot en $\\{W\\}$ | $[m, m, rad]$ |
| $\\dot{q} = [v_x, v_y, \\omega]^T$ | Velocidad del robot en $\\{R\\}$ | $[m/s, m/s, rad/s]$ |
| $\\alpha_i$ | Ángulo de dirección de la rueda $i$ | $rad$ |
| $\\dot{\\phi}_i$ | Velocidad angular de la rueda $i$ | $rad/s$ |

### Índices de Ruedas

$$
i \\in \\{FL, FR, RL, RR\\}
$$

- **FL**: Front Left (Frontal Izquierda)
- **FR**: Front Right (Frontal Derecha)
- **RL**: Rear Left (Trasera Izquierda)
- **RR**: Rear Right (Trasera Derecha)

---

## Estructura del Modelo

```
Sistema 4WD4WS
│
├─ Cinemática Directa: (α₁, α₂, α₃, α₄, φ̇₁, φ̇₂, φ̇₃, φ̇₄) → (vₓ, vᵧ, ω)
│
├─ Cinemática Inversa: (vₓ, vᵧ, ω) → (α₁, α₂, α₃, α₄, φ̇₁, φ̇₂, φ̇₃, φ̇₄)
│  ├─ Modo Omnidireccional
│  ├─ Modo Ackermann
│  └─ Modo Crab
│
└─ Control PID
   ├─ Steering Controllers (4x): α_desired → τ_steering
   └─ Wheel Controllers (4x): φ̇_desired → τ_wheel
```

---

## Ecuaciones Fundamentales

### Relación Velocidad Lineal - Angular

Para cada rueda $i$ con radio $r$:

$$
v_{w_i} = r \\cdot \\dot{\\phi}_i
$$

### Transformada de Velocidad Robot → Mundo

$$
\\begin{bmatrix} \\dot{x} \\\\ \\dot{y} \\\\ \\dot{\\theta} \\end{bmatrix}_W =
\\begin{bmatrix}
\\cos\\theta & -\\sin\\theta & 0 \\\\
\\sin\\theta & \\cos\\theta & 0 \\\\
0 & 0 & 1
\\end{bmatrix}
\\begin{bmatrix} v_x \\\\ v_y \\\\ \\omega \\end{bmatrix}_R
$$

---

## Referencias Rápidas

### Parámetros del Robot Tadeo eCar

| Parámetro | Símbolo | Valor |
|-----------|---------|-------|
| Radio de rueda | $r$ | 0.1 m |
| Distancia entre ejes | $L$ | 1.058 m |
| Ancho de vía | $W$ | 0.55 m |
| Velocidad lineal máxima | $v_{max}$ | 2.0 m/s |
| Velocidad angular máxima | $\\omega_{max}$ | 1.0 rad/s |
| Ángulo de dirección máximo | $\\alpha_{max}$ | 1.57 rad (90°) |

### Implementación

- **Nodo**: `fourws_kinematics_node.py`
- **Frecuencia de control**: 100 Hz
- **Framework**: ros2_control con JointGroupVelocityController

---

## Convenciones

1. **Sistema de coordenadas**: Derecha (x adelante, y izquierda, z arriba)
2. **Ángulos positivos**: Sentido antihorario (regla de la mano derecha)
3. **Velocidades**: Expresadas en el frame del robot $\\{R\\}$ a menos que se indique lo contrario
4. **Notación vectorial**: Columnas por defecto

---

**Autor**: Tadeo Robotics Group
**Fecha**: 2025
**Versión**: 1.0.0
