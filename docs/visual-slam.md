# Visual SLAM: what works and what does not

`tadeocar_vslam` runs RTAB-Map on the ZED 2i's RGB-D stream. This document
exists because the honest answer to "does the visual SLAM work" has two parts.

---

## What works

RTAB-Map builds the map, closes loops, publishes a dense 3D cloud and an
occupancy grid derived from the depth image. Over a 45 m lap of the factory it
tracked ground truth to **0.26 m**, and the accumulated cloud came to 100 270
points.

```bash
ros2 launch tadeocar_bringup vslam_bringup.launch.py
```

---

## What it maps on top of

`odom_source` decides who publishes `odom -> base_footprint`:

| Value | Who owns the transform | Status |
|---|---|---|
| `ekf` (default) | the fused wheel-and-IMU pose | works |
| `visual` | `rgbd_odometry`, camera only | measurably not good enough here |

The default is `ekf`, and that is a measured decision rather than a preference.

---

## Why camera-only odometry is not the default

On its own visual odometry, over the same lap:

- registration **failed on 13 % of frames**, always while the robot was turning
- each failure triggered "Odometry automatically reset", which breaks the
  trajectory's continuity
- the estimate ended **18 m** from ground truth

### The first problem was the world, and it is fixed

A flat-shaded box has an intensity gradient nowhere except at its silhouette,
and a corner detector needs gradients. On the untextured factory world, visual
odometry ran at **0 to 9 inliers** against a threshold of 10 and lost tracking
within seconds of the robot moving.

Three changes addressed that, all of them in the world rather than the tuning:

1. **Textures.** `generate_textures.py` emits seven tileable textures, each
   carrying hard edges — slab joints, panel seams, plank lines, rivets — at a
   scale the camera resolves from a few metres away.
2. **Panelled walls.** One box is one texture tile stretched over the whole
   run, which is a smooth gradient and no help. Long walls are emitted as 2.5 m
   panels so a seam falls every couple of metres.
3. **A roof.** The camera sits 0.30 m off the ground with a 110 degree field of
   view. Without a roof it spent the top third of every frame on empty
   background: 27 % of depth pixels came back non-finite.

With those, inliers went from 0–9 to **53–65**.

### The second problem is not fixed

Even with good inlier counts, registration still fails during turns. What was
tried, and what it did:

| Change | Result |
|---|---|
| `Vis/MinInliers` 10 → 8 | fewer rejections, no improvement in the trajectory |
| `GFTT/MinDistance` 7 → 5, `Kp/MaxFeatures` 400 → 750 | more features, same failures |
| `Vis/CorGuessWinSize` 20 → 40 | intended for exactly this, not enough on its own |
| `OdomF2M/MaxSize` 2000 → 3000 | larger local map, no change |
| `Odom/Strategy` frame-to-map → frame-to-frame | **worse**: 33 % of frames failed |

The pattern that remains is consistent: a 110 degree field of view sweeping at
0.6 rad/s moves a feature further between frames than the correspondence search
expects, and this world has no far-field structure to hold onto while the near
field sweeps past.

### What would probably fix it

In rough order of expected value:

1. **A motion prior from the wheels.** RTAB-Map's `rgbd_odometry` accepts a
   guess through `guess_frame_id`, which needs the wheel odometry published on
   a frame of its own. The obstacle is that a frame has one parent, so this
   needs a second transform tree rather than a parameter change.
2. **More far-field structure in the world**: signage on the walls, gantries,
   anything with texture at 6 to 8 m that stays in view through a turn.
3. **A slower angular velocity limit while visual odometry owns the pose**,
   which is a real mitigation on hardware too.

None of these has been tried. They are listed so the next person does not start
from the beginning.

---

## Running the visual mode anyway

```bash
ros2 launch tadeocar_bringup vslam_bringup.launch.py odom_source:=visual
ros2 launch tadeocar_bringup vslam_bringup.launch.py odom_source:=visual rtabmap_viz:=true
```

`rtabmap_viz` shows the feature matches frame by frame, which is the fastest
way to see a registration failure happen rather than infer it from a log.

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
it — 100 270 points against 3 586. RTAB-Map still projects it down to the 2D
`/map` that Nav2 reads, so nothing downstream loses anything.

**Gazebo's own point cloud is not bridged.** gz-sensors emits an
`rgbd_camera`'s cloud in the sensor's body axes, x forward, while `camera_info`
and the depth image are optical, and one `gz_frame_id` has to label all four
topics. Whichever convention it names, the other is wrong. The cloud ROS sees
is reprojected from the depth image through `camera_info`, which lands it in
the optical frame by construction and is the same projection the ZED SDK
performs on real hardware.
