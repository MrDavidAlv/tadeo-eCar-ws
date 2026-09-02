# Control System of the TadeoeCar Robot

Back to the [mathematical model index](README.md).

---

## 1. Architecture

```
  joystick ──┐
  web UI ────┤                         ┌── cmd_pos ──> 4 steering servos
  teleop ────┼─> twist_mux ─> /cmd_vel ┼
  Nav2 ──────┘        ▲               └── cmd_vel ──> 4 drive motors
                      │                    (fourws_kinematics_node)
                   e_stop
```

One node, `fourws_kinematics_node`, converts a body twist into per-wheel
commands. Nothing else writes to the joints.

The split into two command types is a property of the hardware, not a
preference: the steering axes carry 270 degree industrial servos, and a servo
takes an **angle**; the drive wheels carry BLDC motors through ODrive
controllers, which take a **velocity**.

### 1.1 Priorities

`twist_mux` arbitrates. Higher wins, and a source is dropped after its timeout:

| Source | Topic | Priority | Timeout |
|---|---|---|---|
| Emergency stop | `e_stop` | 255 (lock) | — |
| Joystick / Xbox | `cmd_vel_joy` | 100 | 0.5 s |
| Web interface | `cmd_vel_web` | 90 | 0.5 s |
| Teleop | `cmd_vel_teleop` | 50 | 0.5 s |
| Nav2 | `cmd_vel_nav` | 10 | 0.5 s |

Nav2 is deliberately last. A person reaching for the controller while the robot
is navigating expects to win, and this is where that expectation is honoured.

---

## 2. The steering loop

Each steering joint is closed by a position PID inside Gazebo
(`JointPositionController`), standing in for the servo's own internal loop on
the real robot.

### 2.1 Why the gains are what they are

A steered tyre carrying about 240 N generates a **self-aligning torque** of
roughly 12 N·m as it scrubs. A proportional loop can only oppose a constant
torque by sitting at a constant error, and the integral term is what removes
it — if its limit is high enough to reach the required output.

At the original gains it was not. `i_max` was 10 N·m against a disturbance of
about 12, so the integrator saturated just short of what was needed and the
error never nulled. Measured while driving straight for 10 seconds:

| $k_p$ | $k_i$ | $k_d$ | $i_\text{max}$ | mean error | spread |
|---|---|---|---|---|---|
| 200 | 5 | 30 | 10 | 2.5° | 0.4° |
| 200 | 50 | 30 | 100 | 1.6° | 0.4° |
| 600 | 60 | 60 | 200 | 0.5° | 0.1° |
| **1200** | **100** | **100** | **400** | **0.2°** | **0.1°** |

The visible consequence of the first row was that the robot toed out as it
drove and wandered up to 0.5 m off a 4 m straight line — which looked like a
kinematics bug and was not.

### 2.2 Step response

At the chosen gains, a 90 degree step:

- 0.48 s to 90 % of the commanded angle
- settled inside 1 degree at 0.65 s
- 0.3 % overshoot

The rise time is set by the joint's own 3 rad/s velocity limit, not by the
gains, which is the right thing to be limited by: making the loop stiffer would
only make it saturate harder.

---

## 3. The drive gate

A servo takes tenths of a second to swing 90 degrees. A wheel driven at full
speed through that swing drags sideways across the ground the whole time, which
scrubs the tyre, wastes the command and pushes the robot in directions nobody
asked for.

So wheel velocity is gated on steering error:

$$g = \operatorname{clamp}\!\left(1 - \frac{e_\text{worst} - \varepsilon}{\pi/2 - \varepsilon},\ 0,\ 1\right),
\qquad \varepsilon = 0.20 \text{ rad}$$

with two properties that are both the result of measurement rather than design:

**One gate for all four wheels, taken from the worst error.** Gating each wheel
on its own error was tried first and is wrong: the four servos never finish a
large swing at the same instant, so for a couple of tenths of a second some
wheels drive in the new direction while others still drive in the old one, and
the mismatch is a yaw impulse. It showed up as a crab manoeuvre — the one
command whose entire purpose is to keep the heading fixed — rotating the robot
by 6.8 degrees over 1.8 m. Coupling the gate cut that to under a degree.

**It only engages above 0.25 m/s.** Scrubbing costs in proportion to how fast
the tyre is being dragged, so below a crawl there is nothing to prevent and
closing the gate does real harm. Nav2 sets off at the slowest velocity sample
its acceleration limit allows, about 0.09 m/s, and at that speed the
omnidirectional solution asks for wheel angles of 20 to 40 degrees even though
the body is barely turning. Gating that stopped the robot, which kept its
measured velocity at zero, which kept Nav2 at its slowest sample: a standoff
that had the controller reporting "failed to make progress" while the wheels
aimed and re-aimed. Traced over a 3 m leg, the robot crawled the whole way at
0.09 m/s.

---

## 4. Path following: Regulated Pure Pursuit

Nav2's local controller is
`nav2_regulated_pure_pursuit_controller`, and the choice is worth explaining
because DWB is the more common default.

### 4.1 Why not DWB

DWB samples a window of velocities, rolls each one forward for `sim_time`
seconds and scores the resulting trajectories. That is sound for a differential
or holonomic robot, where any velocity in the window is available immediately.
It is not sound here: a 4WS chassis needs about half a second to swing four
servos before it travels in a new direction, so every trajectory was being
scored against motion the robot would not perform, and the controller reacted
to the mismatch by choosing a different velocity next cycle.

Traced at 2 Hz, it settled into commanding $v_x = 0$, $v_y = 0$ and $\omega_z$
alternating between $+0.09$ and $-0.09$ rad/s **every single cycle**, with the
wheels parked at $-60^\circ$. The robot vibrated in place until the progress
checker aborted the goal.

### 4.2 Why pure pursuit works

Pure pursuit asks a different question — *which way is the path, one lookahead
ahead of me* — and the answer to that question moves smoothly as the robot
moves. A smoothly moving commanded direction is one the servos can follow.

| Parameter | Value | Reason |
|---|---|---|
| `desired_linear_vel` | 0.8 m/s | Below the 1.0 m/s ceiling, with headroom for regulation |
| `lookahead_dist` | 1.4 m | About one robot length |
| `min` / `max_lookahead_dist` | 0.7 / 2.2 m | Velocity-scaled between them |
| `regulated_linear_scaling_min_radius` | 0.9 m | A 1.42 m robot taking a tighter turn at speed sweeps its tail wide |
| `use_rotate_to_heading` | true | This robot can turn on the spot; a car cannot, which is why the option exists |
| `allow_reversing` | false | Only the forward half is instrumented: the LiDAR is 360°, the camera is not |

---

## 5. Costmaps and the footprint

The footprint is the **deck**, $1.418 \times 0.815$ m, not the $1.16 \times
0.65$ m wheel box it used to be. The deck overhangs the wheels on every side,
so Nav2 was planning paths that fit a robot 16 cm narrower than this one.

Inflation follows from it. The inscribed radius is 0.408 m, so an inflation
radius of 0.55 m let the planner route the robot's centre 0.55 m from an
obstacle — which puts the deck's corner 0.15 m from it. Pure pursuit's own
collision check then refused to drive the path its planner had produced, and
the goal failed with the two halves of Nav2 disagreeing.

At 0.85 m the planner keeps a body width of clearance and the factory's 3 m
cross aisle still has 1.3 m of zero-cost corridor down the middle.

The local costmap is 6 × 6 m. At 3 × 3 m, a 1.42 m robot looking 1.4 m ahead
had both ends of its lookahead outside the window.

---

## 6. Localisation: AMCL

`nav2_amcl::OmniMotionModel`, because the robot can translate sideways and a
differential motion model would call that impossible.

The motion noise parameters were 0.05 across the board, which tells AMCL the
odometry is almost exact. It was: `/odom` carried Gazebo's ground-truth pose.
It carries dead reckoning now — measured at roughly 0.3 % over a straight 4 m
and worse through turns — so the particle cloud has to be allowed to spread far
enough to contain that, and $\alpha_{1,2} = 0.15$, $\alpha_{3,4,5} = 0.10$.

`laser_max_range` is 8.0 m, matching the sensor. Declaring 10 asks AMCL to
weigh returns the LiDAR cannot produce.

---

## 7. Measured accuracy

Everything here is against Gazebo's ground-truth pose, which the simulation
publishes on `/odom_truth` for exactly this purpose and which nothing in the
estimation chain is allowed to consume.

### 7.1 Open loop

| Manoeuvre | Commanded | Measured | Error |
|---|---|---|---|
| Straight, 0.5 m/s × 8 s | 4.000 m | 3.98 m | 0.5 %, lateral drift < 15 mm, heading < 0.7° |
| Steering step | 90° | 0.48 s to 90 %, 0.65 s to ±1° | 0.3 % overshoot |
| Crab, 0.4 m/s × 5 s | 2.00 m sideways, 0° | 1.8 m | heading held to 0.6–3.6° |
| Spin in place, 0.5 rad/s × π s | 90° | 70–80° | see below |

The spin shortfall is not a tracking error: the four servos have to swing to
about $\pm 62^\circ$ before any rotation happens at all, and the gate holds the
wheels while they do, so the robot spends less than the full $\pi$ seconds
actually turning. Commanding the same rotation from wheels already in position
reaches it.

### 7.2 Closed loop

| | Result |
|---|---|
| SLAM, 100 m lap | 97 % of mapped cells within 10 cm of true geometry, mean 2.1 cm |
| Fused pose after that lap | 0.55 m, 0.6° from ground truth |
| Wheel odometry after the same lap | 5.2 m, 21° |
| Nav2 | 5 of 5 goals reached, 22–32 cm from the requested pose, 7–22 s each |
| AMCL while driving | 26.5 cm mean position error, 50 cm worst, 2.45° heading |
| RGB-D SLAM, 45 m lap | 0.26 m from ground truth |

---

## 8. Limitations

### 8.1 There is no velocity loop on the wheels

`JointController` with `use_velocity_commands` applies the commanded velocity
directly. There is no torque limit and no current limit, so the simulated robot
accelerates harder than the real one will and never stalls a motor. Anything
that depends on drivetrain dynamics — a ramp too steep to climb, a wheel that
cannot break traction — is not represented.

The yard world's three friction patches are the part of that gap this workspace
can measure: they change what the tyre can transmit even though the drivetrain
is idealised.

### 8.2 Crab heading hold depends on the EKF

The correction in section 3 of [kinematics.md](kinematics.md) reads
`/odometry/filtered`. With `odom_source:=wheel` there is no fused heading, the
hold does not engage, and crab is open loop with the 4–14 degree spread
measured above. That degradation is silent by design — the alternative is a
controller that refuses to work without a filter — but it is worth knowing.

### 8.3 Reversing is not instrumented

`allow_reversing` is false in the planner and the camera faces forward. The
LiDAR does cover 360 degrees, so the robot is not blind behind; it simply has
no depth or colour there, and the deck overhangs the rear axle by 0.18 m.

---

## 9. References

- Regulated Pure Pursuit: Macenski, Moore, Lu, Merzlyakov, Ferguson,
  *From the desks of ROS maintainers: A survey of modern and capable mobile
  robotics algorithms in the open-source Robot Operating System*, 2023.
- AMCL: Fox, *KLD-Sampling: Adaptive Particle Filters*, 2001.
- `robot_localization`: Moore and Stouch, *A Generalized Extended Kalman Filter
  Implementation for the Robot Operating System*, IAS-13, 2014.
