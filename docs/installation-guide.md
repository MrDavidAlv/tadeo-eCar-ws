# Installation Guide

Tested on Ubuntu 22.04 with ROS 2 Humble and Ignition Gazebo Fortress.

---

## 1. ROS 2 Humble

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y ros-humble-desktop
```

## 2. Dependencies

```bash
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rtabmap-ros \
  ros-humble-robot-localization \
  ros-humble-twist-mux \
  ros-humble-joy ros-humble-teleop-twist-joy \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rviz2

pip3 install numpy websockets
```

`numpy` is a hard requirement: the wheel odometry solves an overdetermined
system every cycle. `websockets` is only needed for the web control interface.

Pillow is needed only to regenerate the world textures, which are already
committed. Install it if you intend to run
`tadeocar_gazebo/scripts/generate_textures.py`.

### Gazebo version

This workspace targets **Ignition Fortress**, which is what `ros-humble-ros-gz`
installs. The command is `ign gazebo`, not `gz sim` — the latter belongs to
Garden and later, and answers "Invalid arguments" here. The `gz-sim-*` plugin
filenames in the SDF do resolve under Fortress.

```bash
ign gazebo --version    # expect 6.x
```

## 3. Build

```bash
mkdir -p ~/projects_ros2 && cd ~/projects_ros2
git clone <repository-url> tadeo-eCar-ws
cd tadeo-eCar-ws

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 4. Check it worked

```bash
# The description parses and the URDF and the Gazebo model agree
xacro src/tadeocar_description/urdf/tadeocar.urdf.xacro > /tmp/tadeocar.urdf
check_urdf /tmp/tadeocar.urdf
python3 -m pytest src/tadeocar_description/test/ -q      # 20 tests

# The worlds are valid SDF
ign sdf -k src/tadeocar_gazebo/worlds/factory.world
ign sdf -k src/tadeocar_gazebo/worlds/yard.world

# The demo comes up
ros2 launch tadeocar_bringup demo.launch.py
```

---

## 5. Optional: the physical ZED 2i

`zed_wrapper` is **not** declared in any `package.xml`, deliberately. It is not
in rosdistro — it builds from source against the Stereolabs SDK, which needs
CUDA — so declaring it would break `rosdep install` for anyone who only wants
the simulation.

```bash
# Stereolabs SDK first, from https://www.stereolabs.com/developers/release/
cd ~/projects_ros2/tadeo-eCar-ws/src
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd .. && colcon build --symlink-install
```

Then:

```bash
ros2 launch tadeocar_perception zed2i.launch.py
```

Without the wrapper that launch file prints a sentence explaining what is
missing rather than failing with an ament lookup error.

---

## 6. Troubleshooting

**Gazebo starts with no models, or meshes are missing.**
`GZ_SIM_RESOURCE_PATH` has to include `tadeocar_gazebo/models`.
`simulation.launch.py` sets it; if you launch Gazebo by hand, export it
yourself.

**The IMU topic never appears.** The world has to load
`gz-sim-imu-system`. An `<sensor type="imu">` publishes nothing without it and
Gazebo logs no warning. All three generated worlds load it.

**`ros2 topic echo /map` prints nothing.** `/map`, `/cloud_map` and
`/tf_static` are transient local. Subscribe with matching QoS
(`--qos-durability transient_local`) or you will see an empty topic and
conclude the node is dead.

**`ros2 node list` comes back empty although processes are running.** DDS
discovery state can be left behind by processes that were killed rather than
shut down. Remove `/dev/shm/fastrtps_*` and restart the daemon:

```bash
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ros2 daemon stop && ros2 daemon start
```

**RViz shows the robot as a flat white smear.** The fixed frame is set to a
frame nothing publishes. Use `base_footprint` when only
`robot_state_publisher` is running, or `map` once SLAM is up.
