# Project Structure

Eight packages, each with one job. The dependency direction is downwards:
`tadeocar_bringup` composes, the rest do not know about each other except
through topics.

```
tadeo-eCar-ws/
├── docs/                       this documentation
├── images/                     photographs and diagrams for the README
└── src/
    ├── tadeocar_description/   geometry: URDF, meshes, the URDF/SDF test
    ├── tadeocar_gazebo/        simulation: the Gazebo model, worlds, generators, bridge
    ├── tadeocar_control/       4WS kinematics, wheel odometry, teleoperation
    ├── tadeocar_perception/    EKF state estimation, ZED 2i bringup
    ├── tadeocar_slam/          SLAM Toolbox
    ├── tadeocar_vslam/         RTAB-Map RGB-D visual SLAM
    ├── tadeocar_navigation/    Nav2 configuration and maps
    └── tadeocar_bringup/       the launch files a person actually runs
```

---

## `tadeocar_description`

```
urdf/tadeocar.urdf.xacro     the single source of truth for geometry
meshes/                      chassis, wheels, suspension, ZED 2i
test/test_urdf_sdf_match.py  20 tests that fail the build if the URDF and the
                             Gazebo model stop agreeing
launch/display.launch.py     the robot in RViz with joint sliders, no simulator
```

The description is xacro because the wheel radius appears in eight places and
had the wrong value in all of them. It carries visual and collision geometry
but no inertial tags: physics lives in the Gazebo model.

## `tadeocar_gazebo`

```
models/tadeocar_v1/model.sdf     mass, friction, sensors, controller plugins
models/materials/textures/       seven generated textures
worlds/{factory,yard,empty}.world
scripts/world_common.py          SDF and occupancy-grid emission, layout checks
scripts/generate_*_world.py      one script per world
scripts/generate_textures.py
launch/simulation.launch.py      everything the robot needs to exist, nothing above it
tadeocar_gazebo/odom_to_tf.py    republishes an odometry topic as a transform
```

Each world script emits the world **and** its Nav2 occupancy grid from the same
list of primitives. A world edited by hand and a map edited by hand drift apart
the moment either changes, and the symptom — Nav2 planning around walls that
are not there — looks like a localisation bug.

## `tadeocar_control`

```
config/robot_params.yaml           geometry and limits, loaded by both nodes
config/twist_mux.yaml              command priorities
tadeocar_control/
  fourws_kinematics_node.py        body twist -> steering angles and wheel speeds
  wheel_odometry_node.py           wheel states -> body twist -> pose
  web_control_node.py              websocket bridge for the browser interface
  xbox_control_node.py             gamepad
web/                               the browser interface itself
```

The controller and the odometer load the same parameter file. A controller and
an odometer that disagree about the wheel radius produce a robot that drives
one distance and reports another.

## `tadeocar_perception`

```
config/ekf_sim.yaml     wheels + IMU
config/ekf_real.yaml    wheels + IMU + the ZED SDK's positional tracking
launch/ekf.launch.py    robot_localization, agnostic of sim or hardware
launch/zed2i.launch.py  the physical camera
```

The header of each YAML explains what the other one does differently and why.

## `tadeocar_slam`, `tadeocar_vslam`, `tadeocar_navigation`

One stack each: SLAM Toolbox, RTAB-Map, Nav2. All three are agnostic of
simulation or hardware — they read topics and know nothing about Gazebo.

`tadeocar_navigation/maps/` holds the generated ground truth for each world,
plus whatever you save from a SLAM run.

## `tadeocar_bringup`

```
launch/demo.launch.py                simulation + RViz + web control
launch/slam_bringup.launch.py        simulation + SLAM Toolbox + RViz
launch/navigation_bringup.launch.py  simulation + Nav2 + RViz
launch/vslam_bringup.launch.py       simulation + RTAB-Map + RViz
```

These compose and add nothing of their own. Everything about the robot comes
from `simulation.launch.py`, and none of it is duplicated here.

---

## Topics

### Estimation

| Topic | Type | Published by |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | `wheel_odometry_node` — dead reckoning |
| `/odometry/filtered` | `nav_msgs/Odometry` | the EKF — wheels fused with the IMU |
| `/odom_visual` | `nav_msgs/Odometry` | `rgbd_odometry`, in visual SLAM only |
| `/odom_truth` | `nav_msgs/Odometry` | Gazebo — ground truth, for scoring only |

Exactly one of them owns the `odom -> base_footprint` transform at a time,
chosen by `odom_source`. A frame has one parent, and two publishers of one
transform is a broken tree, not a merged one.

### Sensors

| Topic | Type |
|---|---|
| `/scan` | `sensor_msgs/LaserScan` |
| `/zed/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` |
| `/zed/zed_node/rgb/camera_info` | `sensor_msgs/CameraInfo` |
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` |
| `/zed/zed_node/imu/data` | `sensor_msgs/Imu` |
| `/joint_states` | `sensor_msgs/JointState` |

The camera topics carry the names `zed-ros2-wrapper` uses, so swapping the
simulated camera for the physical one changes nothing above the bridge.

### Commands

| Topic | Type | Note |
|---|---|---|
| `/cmd_vel_joy`, `/cmd_vel_web`, `/cmd_vel_teleop`, `/cmd_vel_nav` | `geometry_msgs/Twist` | inputs to `twist_mux` |
| `/cmd_vel` | `geometry_msgs/Twist` | the winner |
| `/robot_mode` | `std_msgs/String` | `omnidirectional`, `ackermann`, `crab` |
| `/e_stop` | `std_msgs/Bool` | a lock, highest priority |
| `/model/tadeocar/joint/*_steering_joint/cmd_pos` | `std_msgs/Float64` | radians |
| `/model/tadeocar/joint/*_wheel_joint/cmd_vel` | `std_msgs/Float64` | rad/s |

---

## Transform tree

```
map
 └── odom                              SLAM Toolbox, AMCL or RTAB-Map
      └── base_footprint               the EKF, wheel odometry or visual odometry
           └── base_link               fixed, one wheel radius up
                ├── base_scan
                ├── sensor_mast_link
                ├── zed2i_camera_link
                │    ├── zed2i_left_camera_frame
                │    │    └── zed2i_left_camera_optical_frame
                │    ├── zed2i_right_camera_frame
                │    │    └── zed2i_right_camera_optical_frame
                │    └── zed2i_imu_link
                └── {front,rear}_{left,right}_steering_link
                     └── {front,rear}_{left,right}_wheel_link
```

Everything below `base_link` comes from `robot_state_publisher`. The two joints
above it are the interesting ones, and which node owns each is the subject of
the estimation table above.
