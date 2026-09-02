# SLAM Validation

How the mapping and navigation figures in this repository were produced, so
they can be reproduced or disputed.

---

## Why there is a ground truth to compare against

The worlds are generated. `generate_factory_world.py` emits `factory.world`
and `factory_ground_truth.{pgm,yaml}` from the same list of boxes and
cylinders, so the occupancy grid is not an approximation of the building — it
is the building, rasterised at 5 cm.

That makes the comparison a measurement rather than an impression: for every
cell the robot mapped, how far is the nearest cell that is genuinely occupied.

Gazebo's ground-truth pose is published on `/odom_truth` and consumed by
nothing in the estimation chain. It exists to score.

---

## Method

1. Launch the mapping stack headless:

   ```bash
   ros2 launch tadeocar_bringup slam_bringup.launch.py \
     world:=factory headless:=true rviz:=false
   ```

2. Drive a lap covering the building. The runs below used a waypoint pilot
   following `/odom_truth`, at 0.7 m/s, about 100 m of travel over 16 legs.

3. Capture `/map`. It is transient local, so a default-QoS subscription
   receives nothing and appears to be an empty topic.

4. Score each mapped cell against the nearest ground-truth cell, and each
   visible ground-truth surface against the nearest mapped cell. Coverage is
   measured against the **outline** of the ground truth rather than its filled
   interior: a LiDAR sees surfaces, and the generator fills obstacles solid, so
   comparing the two directly would report that the robot failed to map the
   inside of every wall.

---

## Results: LiDAR SLAM

| Metric | Value |
|---|---|
| Mapped cells within 10 cm of true geometry | **97.0 %** |
| Mean error, mapped cell to truth | 2.1 cm |
| Median | 0.0 cm |
| 95th percentile | 7.1 cm |
| Coverage of visible surfaces | 59 % |

Coverage is short of 100 % because a single lap down the aisles never sees
behind the rack rows or the machines. That is a property of the route, not of
the mapper.

### Pose at the end of the lap

| Estimator | Position error | Heading error |
|---|---|---|
| EKF (wheels + IMU) | **0.55 m** | 0.6° |
| Wheel odometry alone | 5.2 m | 21° |

Over roughly 100 m of driving. The ratio is what justifies the filter: the same
wheels, the same world, one extra sensor.

---

## Results: navigation

Five goals across the factory, sent as `NavigateToPose` and scored against
ground truth on arrival:

| Goal | Time | Position error | Heading error |
|---|---|---|---|
| (6.0, 0.0, 0°) | 11.0 s | 21.8 cm | 9.2° |
| (8.5, 0.0, −90°) | 6.7 s | 24.2 cm | 7.5° |
| (0.0, −5.0, 90°) | 21.9 s | 30.1 cm | 8.1° |
| (−6.0, 0.0, 180°) | 11.4 s | 32.0 cm | 11.1° |
| (0.0, 0.0, 0°) | 13.0 s | 22.4 cm | 1.3° |

**5 of 5 reached.** Localisation error while driving averaged 26.5 cm, worst
50 cm, heading 2.45° mean.

The goal tolerance is 0.25 m, and the errors above are slightly larger than
that because the goal checker measures against AMCL's estimate while the table
measures against ground truth. The difference between the two columns *is* the
localisation error.

---

## Results: RGB-D visual SLAM

Over a 45 m lap of the factory:

| Metric | camera-only odometry | mapping on the fused pose |
|---|---|---|
| Final pose error | 0.37 m, 0.9° | 0.06 m |
| Failed registrations | 1 in 400 frames | n/a |
| Feature inliers | median 211, peak 683 | n/a |
| Accumulated 3D cloud | 116 865 points | 100 270 points |

Camera-only odometry did not work at first, and the reason was the world rather
than the algorithm: every texture in it was silently failing to load and the
scene rendered flat. [visual-slam.md](visual-slam.md) has the full account.

---

## Results: the yard world

The second world exists to break the assumption that the ground is flat.
Driving 14.2 m up the north ramp onto the dock platform:

| | ground truth | wheel odometry | EKF |
|---|---|---|---|
| $z$ on the platform | 0.350 m | 0.000 m (**−350 mm**) | 0.352 m (+2 mm) |
| $z$ mid-ramp | 0.192 m | 0.000 m | 0.185 m |
| pitch mid-ramp | −6.72° | 0.00° | −6.72° |
| forward distance | 14.187 m | 14.222 m | 14.202 m |

The wheel odometry is not badly tuned. It reports zero because a wheel encoder
cannot tell forward from up. The generator's own figure for the ramp is 6.72
degrees, which is what the IMU-fused estimate returns to two decimal places.

On flat ground the filter costs nothing: over a 4 m straight run the EKF and
ground truth agree to 10 mm.

---

## Reproducing this

The measurement scripts are not in the repository — they are throwaway harness
code, and keeping them would imply a level of support they do not have. What
they did is described above in enough detail to rewrite them in an afternoon:

- drive a waypoint list using `/odom_truth` as feedback;
- subscribe to `/map` with transient-local durability and write it as a PGM;
- rasterise both grids into one frame and compare with a distance transform;
- send `NavigateToPose` goals and read `/odom_truth` when each completes.
