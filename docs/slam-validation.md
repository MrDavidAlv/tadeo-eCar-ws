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
   following `/odom_truth`, at 0.6 m/s, 68 m of travel over 15 legs, turning
   on the spot at every corner: the manoeuvre that is hardest on wheel
   odometry, and on camera odometry hardest of all. The robot carries two
   LiDARs, each scanning the half it faces, merged into one `/scan`.

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
| Metric | Two half-scans | One 360° scan, same lap |
|---|---|---|
| Mapped cells within 10 cm of true geometry | **95.5 %** | 97.3 % |
| Mean error, mapped cell to truth | 2.2 cm | 1.8 cm |
| Median | 0.0 cm | 0.0 cm |
| 95th percentile | 7.1 cm | 5.0 cm |
| Coverage of visible surfaces | 52 % | 52 % |

The second column is the same lap driven with the single 360 degree LiDAR the
model carried before the upper chassis: two half-scans, blind in the strip
beside the robot, map the building within two points of it. Coverage is short
of 100 % because a single lap down the aisles never sees behind the rack rows
or the machines. That is a property of the route, not of the mapper.

### Pose at the end of the lap

| Estimator | Position error | Heading error |
|---|---|---|
| EKF (wheels + IMU) | **0.29 m** | 0.7° |
| Wheel odometry alone | 2.5 m | 33° |

Over 68 m of driving with fifteen pivots. The ratio is what justifies the
filter: the same wheels, the same world, one extra sensor. The pivots are what
the wheels cannot measure: four tyres scrubbing at once, which the gyro sees
and the encoders do not.

---

## Results: navigation

Five goals across the factory, sent as `NavigateToPose` and scored against
ground truth on arrival:

| Goal | Time | Position error | Heading error |
|---|---|---|---|
| (6.0, 0.0, 0°) | 12.0 s | 26.6 cm | 7.3° |
| (8.5, 0.0, 0°) | 4.6 s | 27.8 cm | 6.8° |
| (0.0, −5.0, −90°) | 19.5 s | 35.6 cm | 8.5° |
| (−6.0, 0.0, 180°) | 16.2 s | 20.2 cm | 8.2° |
| (0.0, 0.0, 0°) | 13.9 s | 23.2 cm | 13.0° |

**5 of 5 reached.** And three in the yard, the last two on the dock platform,
which the planner reaches by the ramp:

| Goal | Time | Position error | Heading error |
|---|---|---|---|
| (−4.0, 4.5, 0°) | 8.7 s | 17.2 cm | 2.5° |
| (6.0, 4.5, 0°) | 14.2 s | 25.8 cm | 2.2° |
| (6.0, −3.0, −90°) | 13.5 s | 22.1 cm | 13.5° |

**3 of 3 reached.** Times are wall clock.

The goal tolerance is 0.25 m, and the errors above are slightly larger than
that because the goal checker measures against AMCL's estimate while the table
measures against ground truth. The difference between the two columns *is* the
localisation error.

---

## Results: RGB-D visual SLAM

Over the same 68 m lap with fifteen pivots, two runs, with the ZED 2i on its
mount on the upper chassis, 0.907 m up and pitched 10 degrees down:

| Metric | camera-only odometry | EKF, same drive |
|---|---|---|
| Final pose error | 3.6 m and 14.6 m | 0.25 m both times |
| Frames with no inliers | 2.3 % and 2.9 % | n/a |
| Feature inliers | median 178 and 158, peak 740 | n/a |

Camera-only odometry loses its estimate a pivot at a time, and where it ends
depends on which pivots it loses. In a straight line it holds. It did not work
at all at first, and the reason was the world rather than the algorithm: every
texture in it was silently failing to load and the scene rendered flat.
[visual-slam.md](visual-slam.md) has the full account.

---

## Results: the yard world

The second world exists to break the assumption that the ground is flat.
Driving 14.2 m up the north ramp onto the dock platform:

| | ground truth | wheel odometry | EKF |
|---|---|---|---|
| $z$ on the platform | 0.350 m | 0.000 m (**−350 mm**) | 0.351 m (+1 mm) |
| $z$ mid-ramp | 0.191 m | 0.000 m | 0.185 m |
| pitch mid-ramp | −6.72° | 0.00° | −6.72° |
| pitch on flat ground | 0.00° | 0.00° | 0.00° |

The wheel odometry is not badly tuned. It reports zero because a wheel encoder
cannot tell forward from up. The generator's own figure for the ramp is 6.72
degrees, which is what the IMU-fused estimate returns to two decimal places,
with the IMU itself pitched 10 degrees on the camera's wedge and taken out
through TF.

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
