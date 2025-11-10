# 📚 Documentación Técnica - Robot Tadeo eCar 4WD4WS

Esta carpeta contiene la documentación técnica completa del robot autónomo Tadeo eCar, incluyendo modelos matemáticos, análisis de sistemas y guías de desarrollo.

---

## 📑 Contenido

### 📐 [Modelo Matemático](./modelo-matematico/)

Documentación completa del modelo matemático del robot 4WD4WS:

- **[README](./modelo-matematico/README.md)**: Introducción general y notación matemática
- **[Cinemática](./modelo-matematico/cinematica.md)**: Modelo cinemático directo e inverso
  - Cinemática de los 3 modos: Omnidireccional, Ackermann, Crab
  - Transformadas de coordenadas
  - Ecuaciones de velocidad y posición
  - Restricciones y límites físicos

- **[Control](./modelo-matematico/control.md)**: Sistema de control PID
  - Control de dirección (steering)
  - Control de velocidad de ruedas
  - Análisis de estabilidad
  - Sintonización de parámetros
  - Frecuencias de operación

- **[Parámetros](./modelo-matematico/parametros.md)**: Parámetros físicos del robot
  - Geometría y dimensiones
  - Propiedades inerciales (masas, tensores de inercia)
  - Límites cinemáticos y dinámicos
  - Especificaciones del sensor LiDAR
  - Parámetros de control PID

- **[Diagrama Excalidraw](./modelo-matematico/modelo-4ws.excalidraw)**: Representación visual
  - Vista superior del robot con 4 ruedas
  - Sistemas de coordenadas
  - Vectores de velocidad
  - Dimensiones L y W
  - Ecuaciones clave

---

## 🎯 Uso de la Documentación

### Para Desarrolladores

Si estás desarrollando nuevos controladores o modificando la cinemática:

1. Lee primero [modelo-matematico/README.md](./modelo-matematico/README.md) para entender la notación
2. Consulta [cinematica.md](./modelo-matematico/cinematica.md) para las ecuaciones específicas
3. Revisa [parametros.md](./modelo-matematico/parametros.md) para valores actuales
4. Usa [control.md](./modelo-matematico/control.md) para entender los lazos de control

### Para Investigadores

Si estás analizando el comportamiento del robot o validando simulaciones:

- **Ecuaciones de movimiento**: [cinematica.md](./modelo-matematico/cinematica.md), sección 3-4
- **Parámetros físicos**: [parametros.md](./modelo-matematico/parametros.md), sección 2-4
- **Función de transferencia**: [control.md](./modelo-matematico/control.md), sección 6
- **Diagrama visual**: [modelo-4ws.excalidraw](./modelo-matematico/modelo-4ws.excalidraw)

### Para Estudiantes

Secuencia de estudio recomendada:

1. **Conceptos básicos**: [modelo-matematico/README.md](./modelo-matematico/README.md)
2. **Geometría del robot**: [parametros.md](./modelo-matematico/parametros.md), sección 1
3. **Cinemática simple**: [cinematica.md](./modelo-matematico/cinematica.md), sección 4.1 (Omnidireccional)
4. **Control básico**: [control.md](./modelo-matematico/control.md), sección 2-3
5. **Modos avanzados**: [cinematica.md](./modelo-matematico/cinematica.md), sección 4.2-4.3

---

## 📊 Resumen de Ecuaciones Clave

### Cinemática Inversa (Omnidireccional)

$$
\alpha_i = \text{atan2}(v_y - \omega \cdot p_{x_i}, v_x + \omega \cdot p_{y_i})
$$

$$
\dot{\phi}_i = \frac{1}{r}\sqrt{(v_x + \omega \cdot p_{y_i})^2 + (v_y - \omega \cdot p_{x_i})^2}
$$

### Control PID

**Dirección**:
$$
\dot{\alpha}_{cmd} = K_{p,s} \cdot e_{\alpha} + K_{d,s} \cdot \dot{e}_{\alpha}
$$

**Ruedas**:
$$
\tau_{wheel} = K_{p,w} \cdot e_{\dot{\phi}}
$$

### Transformada de Velocidad

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}_W =
\begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}_R
$$

---

## 🔧 Herramientas Recomendadas

Para visualizar y editar la documentación:

- **Markdown**: VS Code, Typora, Obsidian
- **LaTeX**: Renderizado automático en GitHub
- **Excalidraw**: https://excalidraw.com (abrir archivo .excalidraw)
- **Ecuaciones**: https://latex.codecogs.com (convertir LaTeX a imagen)

---

## 📝 Convenciones de Documentación

### Notación Matemática

- Vectores: minúsculas en negrita $\mathbf{v}$
- Matrices: mayúsculas en negrita $\mathbf{A}$
- Escalares: minúsculas itálicas $v_x$
- Frames: mayúsculas entre llaves $\{R\}$

### Sistemas de Coordenadas

- $\{W\}$: Sistema mundial (map/odom)
- $\{R\}$: Sistema del robot (base_link)
- $\{W_i\}$: Sistema de rueda $i$

### Índices de Ruedas

- FL: Front Left (Frontal Izquierda)
- FR: Front Right (Frontal Derecha)
- RL: Rear Left (Trasera Izquierda)
- RR: Rear Right (Trasera Derecha)

---

## 🔗 Referencias Cruzadas

| Documento | Implementación en Código |
|-----------|-------------------------|
| [cinematica.md](./modelo-matematico/cinematica.md) | `fourws_kinematics_node.py:95-165` |
| [control.md](./modelo-matematico/control.md) | `ros2_controllers.yaml`, `fourws_kinematics_node.py:200-215` |
| [parametros.md](./modelo-matematico/parametros.md) | `fourws_kinematics_node.py:35-55`, `model.sdf` |

---

## 📞 Contribuciones

Para contribuir a la documentación:

1. Mantén el estilo formal y técnico
2. Usa LaTeX para todas las ecuaciones
3. Incluye referencias al código fuente
4. Agrega ejemplos cuando sea posible
5. Verifica que las ecuaciones rendericen correctamente en GitHub

---

**Organización**: Tadeo Robotics Group
**Fecha de Creación**: 2025
**Última Actualización**: 2025-11-10
**Versión**: 1.0.0
