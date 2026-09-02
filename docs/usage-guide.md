# Usage Guide

Every launch file below takes `world:=factory | yard | empty`, or the path to
any `.world` file.

---

## The demo

```bash
ros2 launch tadeocar_bringup demo.launch.py
```

Gazebo, RViz and the web control interface, with the robot at $(-9.0, 4.5)$ in
the yard world facing the north ramp. Drive forward and watch the fused pose
climb with the robot while the raw wheel odometry stays flat on the ground.

```bash
ros2 launch tadeocar_bringup demo.launch.py use_ekf:=false
```

The same run with dead reckoning owning the transform. On the platform the
robot's own idea of where it is sits 0.35 m below where it obviously is. That
contrast is the reason the yard world exists.

Control is at <http://localhost:8080> — keyboard, virtual joystick and sliders.
An Xbox pad works too:

```bash
ros2 launch tadeocar_control xbox_control.launch.py
```

---

## Mapping with the LiDAR

```bash
ros2 launch tadeocar_bringup slam_bringup.launch.py
ros2 launch tadeocar_bringup slam_bringup.launch.py world:=yard
```

SLAM Toolbox in asynchronous mode. Drive the robot; the map builds in RViz.
Save it when you are done:

```bash
ros2 launch tadeocar_slam save_map.launch.py
```

Measured over a 100 m lap of the factory, against the generated ground truth:
97 % of mapped cells within 10 cm, mean error 2.1 cm. See
[slam-validation.md](slam-validation.md).

---

## Autonomous navigation

```bash
ros2 launch tadeocar_bringup navigation_bringup.launch.py
ros2 launch tadeocar_bringup navigation_bringup.launch.py world:=yard map:=yard
```

Nav2 against the world's generated ground-truth map. Set the goal with the
**2D Goal Pose** tool in RViz, or from the command line:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 6.0, y: 0.0}}}}"
```

To navigate on a map you built yourself rather than the ground truth:

```bash
ros2 launch tadeocar_bringup navigation_bringup.launch.py map:=/path/to/my_map.yaml
```

---

## RGB-D visual SLAM

```bash
ros2 launch tadeocar_bringup vslam_bringup.launch.py
```

RTAB-Map on the ZED 2i. RViz shows two clouds: what the camera sees right now,
reprojected from the depth image, and what RTAB-Map has accumulated.

```bash
# RTAB-Map's own window, with the feature matches frame by frame
ros2 launch tadeocar_bringup vslam_bringup.launch.py rtabmap_viz:=true

# camera-only odometry: nothing but the images says where the robot is
ros2 launch tadeocar_bringup vslam_bringup.launch.py odom_source:=visual

# reuse a map already built instead of starting a new one
ros2 launch tadeocar_bringup vslam_bringup.launch.py localization:=true
```

`odom_source` defaults to `ekf`, which is a measured decision rather than a
preference — [visual-slam.md](visual-slam.md) has the numbers.

---

## Driving modes

The kinematics node takes a mode on `/robot_mode`:

```bash
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: omnidirectional}"
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: ackermann}"
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: crab}"
```

| Mode | What it does |
|---|---|
| `omnidirectional` | Every wheel points along its own velocity: any body twist, exactly. The default, and what Nav2 drives. |
| `ackermann` | Front wheels steer, rear stay straight. For paths a car could also follow. |
| `crab` | All four wheels at one angle: translate without turning. Heading is held by a closed loop on the fused yaw. |

---

## Comparing the estimators

Three estimates of the same trajectory are published at once:

| Topic | What it is |
|---|---|
| `/odom` | wheel odometry, dead reckoning from four wheels and four steering angles |
| `/odometry/filtered` | the EKF, wheels fused with the ZED 2i IMU |
| `/odom_truth` | Gazebo's ground truth, for scoring only |

Nothing in the estimation chain consumes `/odom_truth`. It exists so results
can be scored rather than asserted.

```bash
# watch all three while driving
ros2 topic echo /odom_truth --field pose.pose.position
ros2 topic echo /odometry/filtered --field pose.pose.position
ros2 topic echo /odom --field pose.pose.position
```

---

## Regenerating the worlds

The worlds, their occupancy grids and their textures are all generated:

```bash
python3 src/tadeocar_gazebo/scripts/generate_factory_world.py
python3 src/tadeocar_gazebo/scripts/generate_yard_world.py
python3 src/tadeocar_gazebo/scripts/generate_empty_world.py
python3 src/tadeocar_gazebo/scripts/generate_textures.py     # needs Pillow
```

Each world script emits the SDF **and** the ground-truth map from the same
geometry list, so the two cannot drift apart. The generators refuse to emit a
world whose layout is unusable — a blocked corridor, two overlapping models, or
an obstacle in the blind band between wheel height and the LiDAR plane.

---

## Headless

Every bringup takes `headless:=true` for the simulator and `rviz:=false` for
the viewer, which is what continuous integration and batch measurement want:

```bash
ros2 launch tadeocar_bringup slam_bringup.launch.py headless:=true rviz:=false
```
