# Kinematics of the TadeoeCar 4WD4WS Robot

Back to the [mathematical model index](README.md).

---

## 1. Introduction

The TadeoeCar has four wheels, and every one of them both drives and steers.
That is what 4WD4WS means, and it is the reason none of the standard mobile
robot models fits it:

- It is not differential drive, because the wheels are not fixed fore-and-aft.
- It is not Ackermann, because the rear wheels steer too and because it can
  translate sideways.
- It is not a classic omnidirectional platform either, because the wheels are
  conventional tyres, not Mecanum or Swedish rollers. Sideways motion is
  produced by *pointing* the wheels, not by rolling in two directions at once.

The last distinction is the one that shapes everything below. A Mecanum robot
changes direction the instant the command changes. This one has to physically
swing four servos first, which takes about half a second for a 90 degree
change, and every part of the stack that assumes otherwise gets it wrong. See
[control.md](control.md) for what that costs and how it is handled.

---

## 2. Geometry

### 2.1 Wheel positions

Each wheel module has a vertical **steering axis** and a wheel that rotates
about a horizontal axle. The kinematics is written about the steering axes,
whose positions in `base_link` are:

| Wheel | Symbol | $x$ (m) | $y$ (m) |
|---|---|---|---|
| Front left | FL | $+0.478$ | $+0.275$ |
| Front right | FR | $+0.478$ | $-0.275$ |
| Rear left | RL | $-0.580$ | $+0.275$ |
| Rear right | RR | $-0.580$ | $-0.275$ |

So the wheelbase and track are

$$L = 0.478 - (-0.580) = 1.058 \text{ m}, \qquad T = 2 \times 0.275 = 0.550 \text{ m}$$

### 2.2 The robot is not symmetric about `base_link`

This matters more than it looks. The midpoint of the wheelbase is at

$$x_\text{mid} = \frac{0.478 + (-0.580)}{2} = -0.051 \text{ m}$$

which is 51 mm **behind** the origin of `base_link`. Writing the wheel
positions as $(\pm L/2, \pm T/2)$ — which the controller did until this was
found — therefore places the kinematic centre 51 mm away from the frame every
other part of the stack reports poses in.

The visible symptom is specific: a commanded pure rotation, $v_x = v_y = 0$ and
$\omega_z \neq 0$, comes out as a rotation *about a point 51 mm behind the
robot's origin*, which is a rotation plus a small translation. The robot walks
sideways while spinning on the spot. Two separate numbers, `front_axle_x` and
`rear_axle_x`, are what fix it, and they are what
`tadeocar_control/config/robot_params.yaml` carries.

### 2.3 Mechanical trail

The wheel centre sits 50 mm outboard of its steering axis: ahead of it at the
front, behind it at the rear. This is a real feature of the strut and it is
visible in the URDF, but it is deliberately **not** part of the kinematic
model. A steered wheel's kinematics is written about its steering axis; trail
produces a self-aligning moment, which is a dynamics effect, not a change of
where the wheel is.

Measured, the effect is real but second-order: it is one of the disturbances
the steering loop has to hold against, and section 5 of
[control.md](control.md) gives the numbers.

### 2.4 Wheel radius

$$r = 0.125 \text{ m}$$

measured from `meshes/wheels/wheel.dae`, which is 0.250 m across. This is the
only number that converts wheel rotation into distance travelled, and every
file in the workspace used to say 0.1 m. The consequences were exactly what
that implies: a commanded 1.0 m/s came out as 1.25 m/s, the odometry reported
the 1.0 m/s it had asked for rather than the 1.25 m/s that happened, and
`base_footprint` sat 25 mm underground.

`tadeocar_description/test/test_urdf_sdf_match.py` now fails the build if the
URDF and the Gazebo model ever disagree about it again.

### 2.5 Footprint

The deck overhangs the wheels on every side. Measured from
`meshes/chassis/chasis_base.dae`:

$$1.418 \times 0.815 \times 0.246 \text{ m}, \qquad
x \in [-0.757,\ 0.661], \quad y \in [-0.408,\ 0.408]$$

against a wheel box of only $1.16 \times 0.65$ m. The deck, not the wheels,
is therefore what Nav2's footprint has to describe.

### 2.6 Coordinate frames

Standard ROS convention: $x$ forward, $y$ left, $z$ up, angles positive
counter-clockwise.

- **`base_footprint`** — the ground contact plane under the robot. Every pose
  in this stack is reported for this frame: the wheel odometry, the EKF, SLAM,
  Nav2.
- **`base_link`** — the body frame, one wheel radius above `base_footprint`.
- **`base_scan`** — the LiDAR, at $(0.600,\ 0,\ 0.325)$ in `base_link`, so
  0.450 m above the ground.
- **`zed2i_camera_link`** — the camera mount, at $(0.680,\ 0,\ 0.175)$, so
  0.300 m above the ground.

---

## 3. Forward kinematics: body twist to wheels

### 3.1 The one equation everything follows from

For a rigid body moving in the plane with body twist
$\boldsymbol{\xi} = (v_x, v_y, \omega_z)$, the velocity of the material point
at $\mathbf{p} = (p_x, p_y)$ is

$$\mathbf{v}(\mathbf{p}) = \begin{pmatrix} v_x - \omega_z\, p_y \\
v_y + \omega_z\, p_x \end{pmatrix}$$

Every mode below is a different answer to one question: given that velocity at
each wheel's steering axis, what should that wheel do?

A wheel that is not slipping rolls along its own steering direction. So if the
wheel points along $\mathbf{v}(\mathbf{p}_i)$ and turns at the right rate, it
is not asked to slide sideways at all:

$$\delta_i = \operatorname{atan2}\big(v_y + \omega_z p_{x,i},\ v_x - \omega_z p_{y,i}\big),
\qquad
\omega_i = \frac{\lVert \mathbf{v}(\mathbf{p}_i) \rVert}{r}$$

This is **omnidirectional mode**, and it is the general solution: any
achievable body twist, produced exactly.

### 3.2 Folding the steering angle

The servos travel $\pm 135^\circ$, so an angle outside $[-\pi/2, \pi/2]$ is
reachable the other way round: point the wheel forwards and spin it backwards.

$$(\delta_i, s_i) = \begin{cases}
(\delta_i - \pi,\ -1) & \delta_i > \pi/2 \\
(\delta_i + \pi,\ -1) & \delta_i < -\pi/2 \\
(\delta_i,\ +1) & \text{otherwise}
\end{cases}
\qquad \omega_i \leftarrow s_i\, \omega_i$$

Half the servo travel, the same wheel velocity. It matters: without folding, a
turn asking for $170^\circ$ would be clamped at the $135^\circ$ limit and the
wheel would point somewhere the solution never intended.

### 3.3 Ackermann mode

Front wheels steer, rear wheels stay straight, exactly like a car. The
instantaneous centre of rotation lies on the rear axle's line, at signed radius

$$R = \frac{v_x}{\omega_z}$$

to the left of the robot, and each front wheel points at it:

$$\delta_\text{FL} = \arctan\!\left(\frac{L}{R - T/2}\right), \qquad
\delta_\text{FR} = \arctan\!\left(\frac{L}{R + T/2}\right)$$

The inner wheel turns more sharply than the outer one, which is the whole
content of Ackermann geometry.

> **`arctan`, not `atan2`.** They look interchangeable and are not. For a right
> turn $R$ goes negative and `atan2(L, R)` returns a value near $\pi$ rather
> than a small negative angle, so the wheels tried to fold back on themselves
> and the $135^\circ$ clamp left them at the limit. Right turns in this mode
> were broken from the day the node was written; the symptom was that the robot
> turned left correctly and jammed its steering turning right.

Wheel speeds still come from the general equation of 3.1, so the inner wheels
travel their shorter arc instead of dragging.

### 3.4 Crab mode

All four wheels share one angle, so the body translates without rotating:

$$\delta_i = \operatorname{atan2}(v_y, v_x) \quad \text{for all } i, \qquad
\omega_i = \frac{\sqrt{v_x^2 + v_y^2}}{r}$$

> This mode used to point the front and rear wheels in **opposite** directions.
> That is counter-phase steering — the tightest turn the chassis can make, and
> the exact opposite of a crab walk, whose defining property is that the
> heading does not change.

Four parallel wheels leave the robot **neutrally stable in yaw**: nothing about
the geometry opposes a rotation, so a nudge from an uneven contact patch turns
the chassis and it stays turned. Measured open loop, driving 1.7 m sideways
came out anywhere between 4 and 14 degrees off the heading it started with.

Closing that loop needs a yaw estimate, and it specifically cannot come from
the wheels: during a crab all four turn at the same rate whatever the body is
doing, so the wheel odometry reported 0.1 degrees of rotation while the robot
had actually swung 14. The correction therefore runs off the EKF's fused
heading, and it is applied as counter-phase steering:

$$\delta_\text{front} = \delta + \kappa, \qquad \delta_\text{rear} = \delta - \kappa,
\qquad \kappa = \operatorname{clamp}(k_\psi\, e_\psi,\ \pm 0.35)$$

which is a yaw moment with no net sideways force, so it steers the heading back
without disturbing the translation. With $k_\psi = 2.0$, the same manoeuvre
holds heading to between 0.6 and 3.6 degrees.

---

## 4. Inverse kinematics: wheels to body twist

This is the odometry problem, and on a 4WS robot it is **overdetermined**:
eight measurements — four wheel rates, four steering angles — for three
unknowns.

Each wheel gives two equations:

$$\omega_i r \cos\delta_i = v_x - \omega_z p_{y,i}, \qquad
\omega_i r \sin\delta_i = v_y + \omega_z p_{x,i}$$

Stacked over four wheels, $\mathbf{A}\boldsymbol{\xi} = \mathbf{b}$ with

$$\mathbf{A} = \begin{pmatrix}
1 & 0 & -p_{y,1} \\ 0 & 1 & p_{x,1} \\
\vdots & \vdots & \vdots \\
1 & 0 & -p_{y,4} \\ 0 & 1 & p_{x,4}
\end{pmatrix} \in \mathbb{R}^{8\times3},
\qquad
\mathbf{b} = \begin{pmatrix}
\omega_1 r \cos\delta_1 \\ \omega_1 r \sin\delta_1 \\ \vdots
\end{pmatrix}$$

solved in the least-squares sense:

$$\boldsymbol{\xi} = \mathbf{A}^{+}\, \mathbf{b}$$

**Being overdetermined is the point, not a nuisance.** The wheels disagree
whenever one of them slips, and least squares spreads that disagreement over
all four instead of trusting whichever pair a closed-form solution happened to
pick. $\mathbf{A}$ depends only on where the wheels are, so it is
pseudo-inverted once at startup and each cycle is a single matrix-vector
product.

Note that $\mathbf{A}^{\!\top}\mathbf{A}$ is *not* diagonal:

$$\mathbf{A}^{\!\top}\mathbf{A} = \begin{pmatrix}
4 & 0 & 0 \\ 0 & 4 & \sum p_x \\ 0 & \sum p_x & \sum (p_x^2 + p_y^2)
\end{pmatrix},
\qquad \sum p_x = 2(0.478) + 2(-0.580) = -0.204$$

The coupling between $v_y$ and $\omega_z$ is exactly the asymmetry of section
2.2, appearing again in the estimator.

---

## 5. Pose integration

$$\dot{x} = v_x \cos\theta - v_y \sin\theta, \qquad
\dot{y} = v_x \sin\theta + v_y \cos\theta, \qquad
\dot{\theta} = \omega_z$$

integrated at the midpoint of each step:

$$\bar\theta = \theta_k + \tfrac{1}{2}\omega_z \Delta t$$
$$x_{k+1} = x_k + (v_x \cos\bar\theta - v_y \sin\bar\theta)\Delta t$$
$$y_{k+1} = y_k + (v_x \sin\bar\theta + v_y \cos\bar\theta)\Delta t$$
$$\theta_{k+1} = \operatorname{wrap}(\theta_k + \omega_z \Delta t)$$

Using $\theta_k$ instead of $\bar\theta$ is a first-order scheme whose error
per step is small and whose accumulated error over a lap is an arc-shaped bias
— the trajectory curves consistently one way. The midpoint costs one extra
line.

Steps outside $(0,\ 0.5]$ seconds are discarded: a zero step divides by nothing
useful, and a large one means the clock jumped or messages were dropped, in
which case integrating across the gap invents motion that never happened.

---

## 6. What this model cannot know

Everything above is planar. It has no term for $z$, roll or pitch, and no
amount of tuning gives it one, because a wheel encoder cannot tell forward from
up.

Drive 2.82 m along the yard world's 6.72 degree ramp and the wheels report
2.82 m of $x$ and 0 m of $z$. The truth is 2.80 m of $x$ and 0.35 m of $z$.
Measured on the platform at the top:

| | ground truth | wheel odometry | EKF |
|---|---|---|---|
| $z$ | 0.350 m | 0.000 m (**−350 mm**) | 0.352 m (+2 mm) |
| pitch, mid-ramp | −6.72° | 0.00° | −6.72° |

The 0.35 m is not error to be tuned away. It is a question these sensors were
never able to answer, and it is answered by fusing one that can: an
accelerometer at rest sees gravity, so roll and pitch follow without
integrating anything. That is what `tadeocar_perception` does, and its
configuration files carry the argument in full.

---

## 7. Limits

| Quantity | Value | Where enforced |
|---|---|---|
| $\lvert v_x \rvert, \lvert v_y \rvert$ | 1.0 m/s | `robot_params.yaml`, and DWB/pure pursuit match it |
| $\lvert \omega_z \rvert$ | 1.0 rad/s | same |
| Steering angle | $\pm 2.356$ rad ($\pm 135^\circ$) | URDF, SDF joint limits, and the controller's clamp |
| Steering rate | 3.0 rad/s | SDF joint velocity limit |
| Wheel rate | 60 rad/s | SDF joint velocity limit |

The 1.0 m/s ceiling is not the drivetrain's limit, it is the LiDAR's: the scan
turns at 7 Hz, so at 2 m/s the robot moves 0.29 m between scans and scan
matching has to bridge that gap on odometry alone.

---

Next: [control.md](control.md) — how these commands become forces, and what
the loops around them are worth.
