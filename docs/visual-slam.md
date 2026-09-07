# Visual SLAM

`tadeocar_vslam` runs RTAB-Map on the ZED 2i's RGB-D stream. Camera-only
odometry is the default, and it works:

```bash
ros2 launch tadeocar_bringup vslam_bringup.launch.py
```

Over a 45 m lap of the factory, with nothing but the images deciding where the
robot is:

| Metric | Result |
|---|---|
| Final pose error | **0.37 m**, 0.9° |
| Failed registrations | **1 in 400 frames** |
| Odometry resets | 0 |
| Feature inliers | median 211, peak 683 |
| Accumulated 3D cloud | 116 865 points |

For comparison, on the same lap the EKF — wheels fused with the IMU — ends
0.06 m out. Mapping on the fused pose is available and more accurate, but it
demonstrates less, because the camera is no longer the thing being tested:

```bash
ros2 launch tadeocar_bringup vslam_bringup.launch.py odom_source:=ekf
```

---

## How it got there, because the first version did not work

The first working version of this package tracked nothing. Registration failed
on 13 % of frames, always while the robot was turning, each failure triggering
"Odometry automatically reset", and the estimate ended **18 m** from ground
truth over the same lap.

Every reasonable tuning knob was tried against that and none of them helped:

| Change | Result |
|---|---|
| `Vis/MinInliers` 10 → 8 | fewer rejections, same trajectory |
| `GFTT/MinDistance` 7 → 5, `Kp/MaxFeatures` 400 → 750 | more features, same failures |
| `Vis/CorGuessWinSize` 20 → 40 | intended for exactly this, no change |
| `OdomF2M/MaxSize` 2000 → 3000 | larger local map, no change |
| `Odom/Strategy` frame-to-map → frame-to-frame | **worse**: 33 % of frames failed |

The tuning was not the problem. **The world was flat-shaded**, and a flat-shaded
box has an intensity gradient nowhere except at its silhouette. A corner
detector needs gradients.

### What actually fixed it

Three changes, all in the world rather than in RTAB-Map:

**1. The textures had to load.** They were being emitted into
`models/materials/textures/` and referenced as `materials/textures/x.png` from
the world file. Gazebo resolves a relative texture URI **against the directory
of the file that names it**, not against `GZ_SIM_RESOURCE_PATH`, so the server
was looking in `worlds/materials/textures/` and logging

```
[Err] [SceneManager.cc:862] Unable to find file [materials/textures/concrete.png]
```

for every surface in the world, then rendering it flat. The error scrolls past
during startup and nothing downstream complains — the simulation runs, the
camera publishes, the images simply have nothing in them. The textures live
next to the worlds now.

**2. The building needed a roof, and the roof needed lights.** The camera sits
0.30 m off the ground with a 110 degree field of view. Without a roof it spent
the top third of every frame on empty background: 27 % of depth pixels came
back non-finite. With a roof and no luminaires it spent them on an unlit
ceiling. The factory has twelve bay lights on the same grid as its trusses.

**3. Long walls are emitted as 2.5 m panels.** One box is one texture tile
stretched over the whole run, which is a smooth gradient and no help; panelling
puts a seam every couple of metres.

The measurable difference, same lap, same RTAB-Map configuration:

| | flat-shaded world | textured and lit |
|---|---|---|
| Inliers | 53–65 | median 211, peak 683 |
| Failed registrations | 13 % of frames | 0.25 % |
| Odometry resets over the lap | 34 | 0 |
| Final pose error | 18 m | 0.37 m |

The lesson worth keeping is not about RTAB-Map. It is that a synthetic world is
a *sensor input*, and a visual algorithm tested against an untextured one is
being tested against a scene that could not exist.

---

## Configuration notes worth keeping

**Every RTAB-Map parameter is a string.** `launch_ros` infers a parameter's
type from its Python value, so a bare `False` arrives as a bool and the node
aborts with `InvalidParameterTypeException` before publishing anything. They
are wrapped in `ParameterValue(value_type=str)`.

**`subscribe_rgb` and `subscribe_depth` are alternatives, not companions.**
RGB-D is `subscribe_depth` alone; setting both leaves rtabmap choosing between
two incompatible input configurations at startup.

**`Grid/3D: true`** is why `/cloud_map` is the room rather than a flat slice of
it — over a hundred thousand points against three and a half thousand.
RTAB-Map still projects it down to the 2D `/map` that Nav2 reads.

**Gazebo's own point cloud is not bridged.** gz-sensors emits an
`rgbd_camera`'s cloud in the sensor's body axes, x forward, while `camera_info`
and the depth image are optical, and one `gz_frame_id` has to label all four
topics. Whichever convention it names, the other is wrong. The cloud ROS sees
is reprojected from the depth image through `camera_info`, which lands it in
the optical frame by construction and is the same projection the ZED SDK
performs on real hardware.

---

## The yard, and what actually breaks camera odometry there

Every other number here was measured in the factory. The yard is a different
scene - open sky above, uniform asphalt below, walls twenty metres off - and it
was worth measuring rather than assuming either way.

Driving straight, it is fine. Over 5.9 m of flat apron against generated ground
truth, camera-only odometry is out by 0.11 m, which is about 2 % of the
distance and in the same range as indoors.

**One in-place turn is what breaks it.** Same run, immediately after a 180
degree rotation on the spot:

| | Straight, 5.9 m | After one 180 deg turn |
|---|---|---|
| Wheel odometry | 0.01 m / 0.3 deg | 0.29 m / **14.7 deg** |
| Visual odometry | 0.11 m / 1.6 deg | 0.67 m / **48.6 deg** |
| EKF | 0.01 m / 0.0 deg | 0.10 m / 0.0 deg |

The heading error is the damage. It does not recover, and on the leg back it
turns into position: 7.3 m out by the end of a 16.4 m round trip, against
0.12 m for the EKF over the same path.

Two things go wrong at once and they compound. A 4WS platform turning in place
has all four tyres scrubbing, which is the worst case for wheel odometry and
shows up as the 14.7 degrees above. And the camera sweeps through views that
are mostly sky and bare asphalt, so registration has little to hold: inliers
run to a median of 74 in the yard against 211 in the factory, with 7 frames out
of 300 finding nothing at all. Indoors there is margin for a hard manoeuvre;
outdoors there is not.

So the yard is not a place where camera odometry fails. It is a place where it
has no margin, and an in-place turn spends what margin there is. Map on the
fused pose if the route has pivots in it:

```bash
ros2 launch tadeocar_bringup vslam_bringup.launch.py world:=yard odom_source:=ekf
```

**A correction, since an earlier version of this section said otherwise.** The
first measurement put camera odometry at 23.8 % of path in the yard and called
it unusable. That run was spawned at (-13.0, -8.0), which is 0.5 m from the
centre of `light_pole_sw`: the robot started inside a five metre steel pole and
was shoved out of it by physics before the run began. Nothing reported that,
which is why `simulation.launch.py` now refuses a spawn point that is not clear
in the world's own occupancy grid.

## Watching it work

```bash
# RTAB-Map's own window, with the feature matches frame by frame
ros2 launch tadeocar_bringup vslam_bringup.launch.py rtabmap_viz:=true

# reuse a map already built instead of starting a new one
ros2 launch tadeocar_bringup vslam_bringup.launch.py localization:=true
```

`rtabmap_viz` is the fastest way to see a registration succeed or fail rather
than infer it from a log.
