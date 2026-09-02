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

## Watching it work

```bash
# RTAB-Map's own window, with the feature matches frame by frame
ros2 launch tadeocar_bringup vslam_bringup.launch.py rtabmap_viz:=true

# reuse a map already built instead of starting a new one
ros2 launch tadeocar_bringup vslam_bringup.launch.py localization:=true
```

`rtabmap_viz` is the fastest way to see a registration succeed or fail rather
than infer it from a log.
