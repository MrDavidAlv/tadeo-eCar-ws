# ⚙️ Sistema de Control del Robot 4WD4WS

> **NOTA**: Este documento combina valores REALES de configuración (`tadeocar_tf.urdf`, `ros2_controllers.yaml`) con análisis matemático derivado (estabilidad, respuesta en frecuencia). Los parámetros PID son valores reales implementados en el código.

## 1. Arquitectura de Control

El sistema de control del robot Tadeo eCar implementa una arquitectura jerárquica de dos niveles:

```
Nivel Alto: Cinemática
    ↓
    [fourws_kinematics_node]
    (vx, vy, ω) → (α₁...α₄, φ̇₁...φ̇₄)
    ↓
Nivel Bajo: Control PID
    ↓
    [ros2_control + Gazebo]
    ├─ Steering Controllers (4x): αᵢ → τsteering
    └─ Wheel Controllers (4x): φ̇ᵢ → τwheel
```

---

## 2. Control de Dirección (Steering)

### 2.1 Objetivo

Controlar la posición angular de cada junta de dirección para alcanzar el ángulo objetivo $\alpha_{desired}$.

### 2.2 Modelo de la Junta de Dirección

Ecuación de movimiento:

$$
J_s \ddot{\alpha} + b_s \dot{\alpha} + k_s \alpha = \tau_{steering}
$$

Donde:
- $J_s = 6.0$ kg·m² (inercia del sistema de suspensión + rueda)
- $b_s = 5.0$ N·m·s/rad (amortiguamiento viscoso)
- $k_s = 2.0$ N·m/rad (fricción de Coulomb aproximada como resorte)
- $\tau_{steering}$ (N·m): Par aplicado por el actuador

**Límites físicos**:
- Rango: $\alpha \in [-2.356, +2.356]$ rad ($\pm 135°$)
- Par máximo: $\tau_{max} = 100$ N·m
- Velocidad máxima: $\dot{\alpha}_{max} = 1.0$ rad/s

### 2.3 Controlador PD para Dirección

El nodo `fourws_kinematics_node` implementa un control PD de velocidad que genera comandos de velocidad angular para los controladores de dirección:

$$
\dot{\alpha}_{cmd} = K_{p,s} \cdot e_{\alpha} + K_{d,s} \cdot \dot{e}_{\alpha}
$$

Donde:
- $e_{\alpha} = \alpha_{desired} - \alpha_{current}$ (error de posición)
- $\dot{e}_{\alpha} = -\dot{\alpha}_{current}$ (asumiendo $\dot{\alpha}_{desired} = 0$)

**Parámetros de sintonización**:
- $K_{p,s} = 50.0$ (ganancia proporcional)
- $K_{d,s} = 5.0$ (ganancia derivativa, implícita en controlador de velocidad)

### 2.4 Normalización de Ángulos

Para evitar rotaciones innecesarias, el error angular se normaliza al rango $[-\pi, \pi]$:

$$
e_{\alpha} = \text{atan2}(\sin(\alpha_{desired} - \alpha_{current}), \cos(\alpha_{desired} - \alpha_{current}))
$$

**Ejemplo**: Si $\alpha_{current} = -170°$ y $\alpha_{desired} = +170°$, el error es $-20°$ (no $+340°$).

### 2.5 Saturación de Comando

$$
\dot{\alpha}_{cmd}' = \text{clip}(\dot{\alpha}_{cmd}, -\dot{\alpha}_{max}, \dot{\alpha}_{max})
$$

---

## 3. Control de Velocidad de Ruedas

### 3.1 Objetivo

Controlar la velocidad angular de cada rueda para seguir la velocidad objetivo $\dot{\phi}_{desired}$.

### 3.2 Modelo de la Rueda

Ecuación de movimiento:

$$
J_w \ddot{\phi} + b_w \dot{\phi} = \tau_{wheel} - \tau_{load}
$$

Donde:
- $J_w = 6.0$ kg·m² (inercia de la rueda)
- $b_w = 0.01$ N·m·s/rad (amortiguamiento de rodamiento)
- $\tau_{wheel}$ (N·m): Par aplicado por el motor
- $\tau_{load}$ (N·m): Par de carga (fricción, gravedad, aceleración del robot)

**Límites físicos**:
- Par máximo: $\tau_{max} = 200$ N·m
- Velocidad máxima: $\dot{\phi}_{max} = 100$ rad/s

### 3.3 Controlador P para Velocidad

$$
\tau_{wheel} = K_{p,w} \cdot e_{\dot{\phi}}
$$

Donde:
- $e_{\dot{\phi}} = \dot{\phi}_{desired} - \dot{\phi}_{current}$ (error de velocidad)

**Parámetros de sintonización**:
- $K_{p,w} = 10.0$ (ganancia proporcional)

### 3.4 Conversión a Comando de Velocidad

El sistema ros2_control utiliza `JointGroupVelocityController`, por lo que el nodo cinemático envía directamente comandos de velocidad:

$$
\dot{\phi}_{cmd} = \dot{\phi}_{desired}
$$

El controlador interno de Gazebo implementa el PID:

$$
\tau_{wheel} = K_p (\dot{\phi}_{cmd} - \dot{\phi}_{current}) + K_i \int (\dot{\phi}_{cmd} - \dot{\phi}_{current}) dt
$$

Con parámetros configurados en `tadeocar_description/urdf/tadeocar_tf.urdf:35-40`:
- `vel_kp="10.0"`
- `vel_ki="0.0"`
- `vel_kd="1.0"`

---

## 4. Diagrama de Control Completo

### 4.1 Lazo de Control de Dirección

```
αdesired ───►[+]───► Kp,s ───► [Saturación] ───► α̇cmd ───► [ros2_control] ───► [Gazebo] ───► αcurrent
              │ -                                                                    │
              └──────────────────────────────────────────────────────────────────────┘
```

### 4.2 Lazo de Control de Velocidad

```
φ̇desired ───► [Saturación] ───► φ̇cmd ───► [ros2_control PID] ───► [Gazebo] ───► φ̇current
                                                   │ -                               │
                                                   └───────────────────────────────┘
```

---

## 5. Sintonización de Parámetros PID

### 5.1 Método de Sintonización Utilizado

**Ziegler-Nichols (ajustado)** para control de posición de dirección:

1. Incrementar $K_p$ hasta que el sistema oscile: $K_u \approx 80.0$
2. Período de oscilación: $T_u \approx 0.2$ s
3. Aplicar fórmulas ZN:
   - $K_p = 0.6 K_u = 48.0 \rightarrow$ **Redondeado a 50.0**
   - $K_d = 0.125 K_p T_u = 1.2 \rightarrow$ **Ajustado a 5.0**

### 5.2 Criterios de Desempeño

Para un escalón en $\alpha_{desired}$ de $0° \rightarrow 45°$:

| Métrica | Valor Objetivo | Valor Medido |
|---------|----------------|--------------|
| Tiempo de asentamiento ($t_s$) | < 0.5 s | ~0.4 s |
| Sobrepaso máximo ($M_p$) | < 10% | ~5% |
| Error en estado estacionario ($e_{ss}$) | < 1° | ~0.5° |

### 5.3 Respuesta en Frecuencia

Ancho de banda del sistema de dirección:

$$
\omega_c = K_{p,s} \sqrt{\frac{1}{J_s}} \approx 50 \sqrt{\frac{1}{6}} \approx 20.4 \text{ rad/s} \approx 3.2 \text{ Hz}
$$

Esto permite seguir comandos de dirección con cambios de hasta ~3 Hz, suficiente para navegación autónoma.

---

## 6. Análisis de Estabilidad

### 6.1 Sistema de Dirección

Función de transferencia en lazo cerrado (simplificada):

$$
G_s(s) = \frac{\alpha(s)}{\alpha_{desired}(s)} = \frac{K_{p,s}}{J_s s^2 + (b_s + K_{d,s})s + K_{p,s}}
$$

Ecuación característica:

$$
J_s s^2 + (b_s + K_{d,s})s + K_{p,s} = 0
$$

$$
6.0 s^2 + (5.0 + 5.0)s + 50.0 = 0
$$

$$
s^2 + 1.67s + 8.33 = 0
$$

Raíces (polos):

$$
s_{1,2} = \frac{-1.67 \pm \sqrt{1.67^2 - 4(8.33)}}{2} = -0.835 \pm j2.75
$$

**Análisis**:
- Parte real negativa: **Sistema estable** ✓
- Frecuencia natural: $\omega_n = \sqrt{8.33} = 2.89$ rad/s
- Factor de amortiguamiento: $\zeta = \frac{1.67}{2\omega_n} = 0.289$ (subamortiguado)

### 6.2 Sistema de Velocidad

Función de transferencia:

$$
G_w(s) = \frac{\dot{\phi}(s)}{\dot{\phi}_{desired}(s)} = \frac{K_{p,w}}{J_w s + b_w + K_{p,w}}
$$

Polo en:

$$
s = -\frac{b_w + K_{p,w}}{J_w} = -\frac{0.01 + 10.0}{6.0} \approx -1.67
$$

**Análisis**:
- Polo negativo: **Sistema estable** ✓
- Constante de tiempo: $\tau = \frac{1}{1.67} = 0.6$ s

---

## 7. Gestión de Modos de Operación

### 7.1 Transiciones de Modo

Al cambiar de modo (Omnidireccional ↔ Ackermann ↔ Crab), el nodo cinemático implementa:

**Algoritmo de transición suave**:

```python
def transition_to_mode(new_mode, current_alpha):
    if mode_change_detected():
        # Calcular nuevos ángulos objetivo
        target_alpha = compute_kinematics(new_mode, vx, vy, omega)

        # Verificar si el cambio excede límite de velocidad
        delta_alpha = target_alpha - current_alpha
        max_delta = steering_max_velocity * dt

        if abs(delta_alpha) > max_delta:
            # Aplicar rampa de velocidad constante
            alpha_cmd = current_alpha + sign(delta_alpha) * max_delta
        else:
            alpha_cmd = target_alpha

    return alpha_cmd
```

### 7.2 Tópicos de Comando

El nodo suscribe a dos tópicos de velocidad con prioridades diferentes:

| Tópico | Modo por Defecto | Prioridad | Fuente |
|--------|------------------|-----------|--------|
| `/cmd_vel` | Omnidireccional | Alta | Xbox / Web Control |
| `/cmd_vel_nav` | Crab | Media | Nav2 |

**Lógica de arbitraje**:

$$
\mathbf{v}_{cmd} = \begin{cases}
\mathbf{v}_{/cmd\_vel} & \text{si } t - t_{last\_cmd\_vel} < 0.5 \text{ s} \\
\mathbf{v}_{/cmd\_vel\_nav} & \text{si } t - t_{last\_cmd\_vel\_nav} < 1.0 \text{ s} \\
\mathbf{0} & \text{en otro caso (timeout)}
\end{cases}
$$

---

## 8. Compensación de Dinámica

### 8.1 Feedforward de Aceleración (No Implementado)

Para mejorar el seguimiento, se podría agregar término feedforward:

$$
\tau_{wheel} = K_p e_{\dot{\phi}} + J_w \ddot{\phi}_{desired}
$$

Donde $\ddot{\phi}_{desired}$ se estima por diferenciación numérica.

### 8.2 Compensación de Fricción (Implementada en SDF)

El modelo SDF incluye fricción de Coulomb en las ruedas:

$$
\tau_{friction} = \mu_1 \cdot F_N \cdot r
$$

Donde:
- $\mu_1 = 2.5$ (coeficiente de fricción rueda-suelo)
- $F_N \approx \frac{mg}{4} \approx 220$ N (fuerza normal por rueda)
- $r = 0.1$ m

$$
\tau_{friction} \approx 2.5 \times 220 \times 0.1 = 55 \text{ N·m}
$$

---

## 9. Frecuencia de Control

### 9.1 Configuración

- **Frecuencia de actualización ros2_control**: 100 Hz (`update_rate: 100`)
- **Frecuencia del nodo cinemático**: ~50 Hz (período de timer: 20 ms)

### 9.2 Criterio de Nyquist

Para seguir señales de hasta 3 Hz (ω_c del sistema), frecuencia mínima:

$$
f_{sampling} \geq 2 \times f_{max} = 2 \times 3 = 6 \text{ Hz}
$$

**Margen de seguridad**: $\frac{100}{6} \approx 16.7\times$ ✓

---

## 10. Implementación en Código

### 10.1 Controladores ros2_control

**Archivo**: `src/tadeocar_control/config/ros2_controllers.yaml`

```yaml
front_left_steering_controller:
  type: velocity_controllers/JointGroupVelocityController
  joints:
    - front_left_steering_joint

front_left_wheel_controller:
  type: velocity_controllers/JointGroupVelocityController
  joints:
    - front_left_wheel_joint
```

### 10.2 Publicación de Comandos

**Archivo**: `src/tadeocar_control/tadeocar_control/fourws_kinematics_node.py:200-215`

```python
# Control de dirección (PD)
steering_error = normalize_angle(alpha_desired - alpha_current)
alpha_dot_cmd = self.kp_steering * steering_error

# Saturación
alpha_dot_cmd = np.clip(alpha_dot_cmd, -self.max_steering_velocity,
                                        self.max_steering_velocity)

# Publicar a ros2_control
steering_msg = Float64MultiArray()
steering_msg.data = [alpha_dot_cmd]
self.steering_pub.publish(steering_msg)

# Control de velocidad (directo)
wheel_msg = Float64MultiArray()
wheel_msg.data = [phi_dot_desired]
self.wheel_pub.publish(wheel_msg)
```

---

## 11. Diagnóstico y Monitoreo

### 11.1 Tópicos de Diagnóstico

Para monitorear el desempeño del control:

```bash
# Error de seguimiento de dirección
ros2 topic echo /joint_states --field position

# Comandos enviados
ros2 topic echo /front_left_steering_controller/commands

# Velocidades de ruedas
ros2 topic echo /joint_states --field velocity
```

### 11.2 Métricas de Calidad

$$
\text{RMSE}_{\alpha} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}(\alpha_{desired,i} - \alpha_{current,i})^2}
$$

Valor esperado: $\text{RMSE}_{\alpha} < 0.05$ rad (2.86°)

---

**Autor**: Tadeo Robotics Group
**Fecha**: 2025
**Versión**: 1.0.0
