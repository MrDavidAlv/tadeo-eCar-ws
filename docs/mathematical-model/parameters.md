# Physical Parameters of the TadeoeCar Robot

Back to the [mathematical model index](README.md).

Every number here is traceable to a file. Where a value is an assumption rather
than a measurement, it says so — that distinction is the point of the document.

---

## 1. Geometry

### 1.1 Wheels

| Parameter | Value | Source |
|---|---|---|
| Radius $r$ | 0.125 m | measured from `meshes/wheels/wheel.dae`, 0.250 m across |
| Width | 0.200 m | same mesh |
| Mechanical trail | 0.050 m | URDF wheel joint origin, outboard of the steering axis |

The radius is the only number that converts rotation into distance. It appears
in the xacro once and is propagated from there;
`test_urdf_sdf_match.py::test_wheel_radius_matches_the_mesh` pins it.

### 1.2 Steering axes

| Wheel | $x$ (m) | $y$ (m) |
|---|---|---|
| Front left / right | $+0.478$ | $\pm 0.275$ |
| Rear left / right | $-0.580$ | $\pm 0.275$ |

Wheelbase $L = 1.058$ m, track $T = 0.550$ m. The wheelbase midpoint is 51 mm
behind `base_link`; see [kinematics.md](kinematics.md) §2.2 for why that is not
a rounding detail.

### 1.3 Deck

| Parameter | Value |
|---|---|
| Size | $1.418 \times 0.815 \times 0.246$ m |
| Extent in `base_link` | $x \in [-0.757, 0.661]$, $y \in [\pm 0.408]$ |
| Top face above ground | 0.389 m |
| Ground clearance | 0.125 m (wheel radius) |

Measured from `meshes/chassis/chasis_base.dae`. This is the Nav2 footprint.

### 1.4 Sensor mounts

| Frame | Position in `base_link` | Height above ground |
|---|---|---|
| `base_scan` (LiDAR) | $(0.600,\ 0,\ 0.325)$ | 0.450 m |
| `zed2i_camera_link` | $(0.680,\ 0,\ 0.175)$ | 0.300 m |

Each is fixed by one constraint. The LiDAR clears the deck's top face by 61 mm,
which is what buys a full 360 degree sweep with no self-hits. The camera stays
below the LiDAR plane so it never occludes it, and ahead of the front tyre's
sweep, which reaches $x = 0.653$ m.

---

## 2. Mass and inertia

| Body | Mass (kg) | Notes |
|---|---|---|
| Deck (`base_link`) | 50.0 | includes batteries and electronics |
| Steering module × 4 | 6.0 each | strut, servo, hub motor |
| Wheel × 4 | 6.0 each | tyre and rim |
| LiDAR | 0.2 | |
| Sensor mast | 0.3 | |
| ZED 2i | 0.175 | datasheet |
| **Total** | **98.7** | |

Inertia tensors are computed as uniform solids of the corresponding primitive:
a box for the deck and the steering modules, a cylinder about its axle for the
wheels.

$$I_{yy}^\text{wheel} = \tfrac{1}{2} m r^2 = 0.0469,\qquad
I_{xx} = I_{zz} = \tfrac{1}{4} m r^2 + \tfrac{1}{12} m w^2 = 0.0434 \ \text{kg m}^2$$

**These are assumptions.** The real robot's mass distribution has not been
measured, and a 98.7 kg figure built from plausible component masses is a
starting point, not a specification. What it affects is how hard the steering
loop has to work and how quickly the robot settles after a disturbance; it does
not affect any of the kinematic results.

---

## 3. Friction and contact

| Surface | $\mu$ | Where |
|---|---|---|
| Tyre | 1.0 | `model.sdf` wheel collisions |
| Deck | 0.6 | so a collision slides rather than sticks |
| Cement (factory floor, ramps) | 1.00 | `world_common.py` |
| Asphalt (yard apron) | 0.90 | |
| Gravel patch | 0.55 | |
| Sand patch | 0.30 | |

ODE contact parameters on the tyres: $k_p = 10^6$, $k_d = 100$,
`min_depth` 0.001.

The three yard patches carry genuinely different coefficients, not just
different colours. On a 4WS robot that matters twice over: a crab manoeuvre
asks all four tyres to roll in a direction the chassis is not pointing, so
lateral grip decides whether the commanded direction is the one travelled.

---

## 4. Actuator limits

| Quantity | Value | Where enforced |
|---|---|---|
| Steering range | $\pm 2.356$ rad ($\pm 135^\circ$) | URDF and SDF joint limits |
| Steering rate | 3.0 rad/s | SDF joint velocity limit |
| Steering effort | 100 N·m | SDF |
| Wheel rate | 60 rad/s | SDF |
| Wheel effort | 200 N·m | SDF |
| Body linear speed | 1.0 m/s | `robot_params.yaml` |
| Body angular speed | 1.0 rad/s | `robot_params.yaml` |

Steering PID: $k_p = 1200$, $k_i = 100$, $k_d = 100$, $i_\text{max} = 400$.
The sweep that produced those is in [control.md](control.md) §2.1.

---

## 5. LiDAR: YDLIDAR X2

| Parameter | Value |
|---|---|
| Range | 0.12 – 8.0 m |
| Field of view | 360° |
| Samples per revolution | 428 |
| Scan rate | 7 Hz |
| Ranging rate | 3000 samples/s (= 428 × 7) |
| Range noise | Gaussian, $\sigma = 0.015$ m |
| Frame | `base_scan`, 0.450 m above ground |

These are the datasheet figures for the sensor the robot carries. The model
previously claimed 3.5 m at 20 Hz over 320°, and each of those three was wrong
in a way that mattered: 3.5 m in a 20 × 15 m building leaves scan matching
nothing to hold onto along the length of the floor, 20 Hz overstates how much
data arrives, and the 320° clip existed only to hide the robot reading its own
chassis.

The 7 Hz scan rate is what sets the 1.0 m/s speed ceiling: at 2 m/s the robot
moves 0.29 m between scans.

---

## 6. Camera: Stereolabs ZED 2i

| Parameter | Value |
|---|---|
| Resolution | 672 × 376 (the camera's VGA mode) |
| Frame rate | 15 Hz |
| Horizontal FOV | 110° (1.919862 rad) |
| $f_x = f_y$ | 235.27 px |
| $c_x, c_y$ | 336, 188 |
| Depth range | 0.3 – 20 m |
| Baseline | 0.12 m |
| IMU | Bosch BMI088, 400 Hz |

$f_x$ follows from the FOV rather than being set:
$f_x = (672/2)/\tan(55^\circ) = 235.27$.

Simulated as a single `rgbd_camera`. Two 1280 × 720 cameras — what the model
carried before — put the simulation at 8 % of real time and gave RTAB-Map a
stereo pair to match by hand, when Gazebo already renders the depth image that
the ZED SDK computes on the real camera's GPU.

### 6.1 IMU noise

Angular velocity $\sigma = 2\times10^{-4}$ rad/s, linear acceleration
$\sigma = 0.017$ m/s², both Gaussian, no bias term.

**These are assumptions** taken from the BMI088's datasheet class, not
measurements of the part in this camera. If the hardware turns out to behave
differently, these and the `imu0` covariances in
`tadeocar_perception/config/*.yaml` have to move together.

---

## 7. Navigation

| Parameter | Value | Note |
|---|---|---|
| Footprint | $1.418 \times 0.815$ m | the deck |
| Inscribed radius | 0.408 m | |
| Inflation radius | 0.85 m | a body width of clearance |
| Cost scaling factor | 2.5 | |
| Local costmap | 6 × 6 m, 0.05 m | rolling |
| Global costmap resolution | 0.05 m | |
| `xy_goal_tolerance` | 0.25 m | |
| `yaw_goal_tolerance` | 0.25 rad | |
| Obstacle / raytrace range | 7.0 / 8.0 m | matched to the LiDAR |

---

## 8. SLAM

| Parameter | Value |
|---|---|
| Resolution | 0.05 m |
| `max_laser_range` | 8.0 m |
| `minimum_travel_distance` | 0.20 m |
| `minimum_travel_heading` | 0.15 rad |
| `minimum_time_interval` | 0.15 s |
| `loop_search_maximum_distance` | 10.0 m |
| `distance` / `angle_variance_penalty` | 0.3 / 0.6 |

A node every 0.2 m is roughly five per body length: dense enough that
consecutive scans overlap heavily, sparse enough that the graph does not
outgrow the optimiser.

---

## 9. State estimation

The EKF fuses:

| Quantity | Source |
|---|---|
| roll, pitch | ZED 2i IMU, absolute, from gravity |
| roll / pitch / yaw rate | ZED 2i gyro |
| $v_x$, $v_y$ | wheel odometry |
| $x$, $y$, $z$, yaw | nobody — dead reckoned by the filter |

with `two_d_mode: false`, which is the entire point: with it true,
`robot_localization` forces $z$, roll and pitch to zero and the filter becomes
an expensive way to reproduce the flat-world assumption it exists to remove.

On the real robot, `ekf_real.yaml` adds the ZED SDK's positional tracking as an
absolute source for $x$, $y$, $z$ and yaw. There is no Gazebo equivalent of it.

### 9.1 Covariances

`process_noise_covariance` and `initial_estimate_covariance` are left at
`robot_localization`'s defaults, and that is a decision rather than an
oversight. The values that belong there depend on how much the tyres slip,
which depends on the surface — and the yard world's asphalt, gravel and sand
patches exist precisely so that this can be measured rather than guessed.

The measurement would be: drive a known distance on each patch, compare the
integrated wheel odometry against `/odom_truth`, and derive the velocity
variance per surface from the residual. Until that is done, writing a
hand-tuned 15 × 15 matrix would look authoritative and be backed by nothing.

The odometry message's own twist covariances are set — $\sigma_{v_x} = 0.05$,
$\sigma_{v_y} = 0.10$, $\sigma_{\omega_z} = 0.05$ — on the assumption of
roughly 5 % slip on a driven wheel, with lateral velocity trusted half as much
because it is the component a steered wheel scrubs away first. Pose covariances
are set to $10^3$, which is what makes "fuse the twist, ignore the dead-reckoned
pose" true in the numbers and not only in the configuration.

---

## 10. Derived quantities

| Quantity | Expression | Value |
|---|---|---|
| Wheel circumference | $2\pi r$ | 0.7854 m |
| Wheel rate for 1 m/s | $1/r$ | 8.0 rad/s |
| Ramp angle, yard | $\arctan\!\big((0.35 - t)/2.8\big)$ | 6.72° |
| Breakover angle | $2\arctan(2c/L)$, $c = 0.125$ | 26.5° |
| Turning radius, Ackermann at full lock | $L/\tan(135°)$ | see note |
| Minimum turning radius, omni | 0 (spins in place) | |

At full steering lock the Ackermann expression is degenerate — the mode is
meant for car-like paths, and a turn that tight is what omnidirectional mode is
for.

The breakover angle is comfortably above the 6.72° ramp, so the chassis cannot
ground out on the transition at the top.
