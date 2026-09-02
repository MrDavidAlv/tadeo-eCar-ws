#!/usr/bin/env python3
"""Everything the robot needs to exist in simulation, and nothing above it.

Gz Sim, the ROS bridge, robot_state_publisher, the 4WS controller, twist_mux
and one odometry source. SLAM, navigation and visual SLAM are layered on top of
this file by tadeocar_bringup and never duplicate any of it.

Two arguments decide the shape of the run:

  world         factory | yard | empty, or a path to any .world file
  odom_source   ekf | wheel | none

``odom_source`` exists because odom -> base_footprint has exactly one parent
and three different nodes are capable of publishing it. ``ekf`` fuses the
wheels with the ZED 2i's IMU and is the default; ``wheel`` uses dead reckoning
alone, which is what makes the difference measurable; ``none`` leaves the
transform to a node outside this file, which is how the visual SLAM demo hands
it to RTAB-Map's visual odometry.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Where the robot starts, per world. The factory is happy at its origin; the
# yard's origin is up on the dock platform, which is a silly place to begin a
# lap of a circuit whose point is driving up to it.
SPAWN = {
    'factory': (0.0, 0.0, 0.02, 0.0),
    'yard': (-9.0, 4.5, 0.02, 0.0),
    'empty': (0.0, 0.0, 0.02, 0.0),
}


def resolve(context, *args, **kwargs):
    """Resolve the world and the spawn pose in the PARENT launch context.

    IncludeLaunchDescription evaluates its launch_arguments inside the scope of
    the file being included, and in dictionary order, so an argument whose
    value reads LaunchConfiguration('world') sees whatever the earlier 'world'
    entry just wrote there - the fully expanded path, not the short name. Spawn
    coordinates derived that way fall through to their default branch without a
    word of warning and the robot appears at the origin of whatever world it
    was given. Resolving here, once, in the parent context, is the fix.
    """
    pkg_gazebo = get_package_share_directory('tadeocar_gazebo')
    pkg_description = get_package_share_directory('tadeocar_description')
    pkg_control = get_package_share_directory('tadeocar_control')
    pkg_perception = get_package_share_directory('tadeocar_perception')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_arg = LaunchConfiguration('world').perform(context)
    if world_arg.endswith('.world') or os.path.sep in world_arg:
        world_path = world_arg
        world_name = os.path.splitext(os.path.basename(world_arg))[0]
    else:
        world_name = world_arg
        world_path = os.path.join(pkg_gazebo, 'worlds', f'{world_arg}.world')
    if not os.path.exists(world_path):
        raise RuntimeError(
            f'world "{world_arg}" not found at {world_path}. '
            f'Known worlds: {", ".join(sorted(SPAWN))}')

    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_source = LaunchConfiguration('odom_source').perform(context)
    if odom_source not in ('ekf', 'wheel', 'none'):
        raise RuntimeError(
            f'odom_source must be ekf, wheel or none, not "{odom_source}"')

    default_x, default_y, default_z, default_yaw = SPAWN.get(
        world_name, (0.0, 0.0, 0.02, 0.0))

    def spawn_value(name, fallback):
        given = LaunchConfiguration(name).perform(context)
        return given if given else str(fallback)

    spawn_x = spawn_value('spawn_x', default_x)
    spawn_y = spawn_value('spawn_y', default_y)
    spawn_z = spawn_value('spawn_z', default_z)
    spawn_yaw = spawn_value('spawn_yaw', default_yaw)

    headless = LaunchConfiguration('headless').perform(context)
    gz_args = ['-r ', '-s ' if headless.lower() in ('true', '1') else '', world_path]

    sdf_file = os.path.join(pkg_gazebo, 'models', 'tadeocar_v1', 'model.sdf')
    xacro_file = os.path.join(pkg_description, 'urdf', 'tadeocar.urdf.xacro')
    robot_params = os.path.join(pkg_control, 'config', 'robot_params.yaml')

    # Gazebo finds the model's meshes through this, not through package://.
    for path in (os.path.join(pkg_gazebo, 'models'),
                 os.path.join(pkg_gazebo, 'worlds')):
        os.environ['GZ_SIM_RESOURCE_PATH'] = (
            os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + path).strip(':')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    spawn_entity = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-file', sdf_file, '-name', 'tadeocar',
                   '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw])

    # ROS <-> Gz bridge.
    #
    # /zed2i/points is deliberately absent. gz-sensors emits an rgbd_camera's
    # cloud in the sensor's body axes, x forward, while camera_info and the
    # depth image are optical, and a single gz_frame_id has to label all four
    # topics. Whichever convention it names, the other one is wrong: tagged
    # optical, RViz applies the URDF's -90/0/-90 a second time and the cloud
    # lies on its side. The cloud ROS sees is reprojected from the depth image
    # below instead, which lands it in the optical frame by construction.
    #
    # Topic names on the ROS side match zed-ros2-wrapper so that swapping the
    # simulated camera for the real one changes nothing above this line.
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/odom_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/zed2i/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/zed2i/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed2i/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed2i/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ] + [
            f'/model/tadeocar/joint/{w}_steering_joint/cmd_pos'
            '@std_msgs/msg/Float64]gz.msgs.Double'
            for w in ('front_left', 'front_right', 'rear_left', 'rear_right')
        ] + [
            f'/model/tadeocar/joint/{w}_wheel_joint/cmd_vel'
            '@std_msgs/msg/Float64]gz.msgs.Double'
            for w in ('front_left', 'front_right', 'rear_left', 'rear_right')
        ],
        remappings=[
            ('/zed2i/image', '/zed/zed_node/rgb/image_rect_color'),
            ('/zed2i/depth_image', '/zed/zed_node/depth/depth_registered'),
            ('/zed2i/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('/zed2i/imu', '/zed/zed_node/imu/data'),
        ])

    # The point cloud, reprojected from the depth image through camera_info.
    # Reading the intrinsics is what puts it in the optical frame, the
    # convention every ROS point cloud consumer assumes, and it is the same
    # projection the ZED SDK runs on the real camera.
    points = Node(
        package='rtabmap_util', executable='point_cloud_xyzrgb',
        name='zed2i_points', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # RGB and depth are rendered by one sensor but published as two
            # messages, so their stamps only line up approximately.
            'approx_sync': True,
            # Half resolution, 336 x 188. Projecting all 672 x 376 pixels buys
            # nothing a viewer can see and costs real time factor.
            'decimation': 2,
            'voxel_size': 0.0,
            'min_depth': 0.3,
            'max_depth': 20.0,
        }],
        remappings=[
            ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
            ('depth/image', '/zed/zed_node/depth/depth_registered'),
            ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('cloud', '/zed/zed_node/point_cloud/cloud_registered'),
        ],
        condition=IfCondition(LaunchConfiguration('publish_points')))

    # The description is xacro since the wheel radius stopped being written out
    # eight times. Handing robot_state_publisher the raw file gives it a
    # document full of ${...} that KDL rejects, and the RobotModel in RViz then
    # renders as nothing at all without saying why.
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': xacro.process_file(xacro_file).toxml(),
        }])

    fourws_kinematics = Node(
        package='tadeocar_control', executable='fourws_kinematics',
        name='fourws_kinematics_node', output='screen',
        parameters=[robot_params, {'use_sim_time': use_sim_time}])

    # Dead reckoning always runs, because /odom is what SLAM and Nav2 read and
    # because the EKF needs a velocity source. Whether it also owns the
    # transform is the odom_source question.
    wheel_odometry = Node(
        package='tadeocar_control', executable='wheel_odometry',
        name='wheel_odometry_node', output='screen',
        parameters=[robot_params, {
            'use_sim_time': use_sim_time,
            'publish_tf': odom_source == 'wheel',
        }])

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'ekf.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_tf': 'true',
        }.items()) if odom_source == 'ekf' else None

    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        output='screen',
        parameters=[os.path.join(pkg_control, 'config', 'twist_mux.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel')])

    actions = [gz_sim, spawn_entity, bridge, points, robot_state_publisher,
               twist_mux, fourws_kinematics, wheel_odometry]
    if ekf is not None:
        actions.append(ekf)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the simulation clock'),
        DeclareLaunchArgument(
            'world', default_value='factory',
            description='factory, yard, empty, or a path to a .world file'),
        DeclareLaunchArgument(
            'headless', default_value='false', choices=['true', 'false'],
            description='Run Gz Sim without its GUI'),
        DeclareLaunchArgument(
            'odom_source', default_value='ekf', choices=['ekf', 'wheel', 'none'],
            description='Who publishes odom -> base_footprint. ekf fuses the '
                        'wheels with the ZED 2i IMU; wheel is dead reckoning '
                        'alone; none leaves the transform to a node outside '
                        'this file, such as visual odometry'),
        DeclareLaunchArgument(
            'publish_points', default_value='true', choices=['true', 'false'],
            description='Reproject the ZED depth image into a point cloud. '
                        'False when the consumer reads the depth image itself'),
        DeclareLaunchArgument('spawn_x', default_value='',
                              description='Override the per-world spawn X'),
        DeclareLaunchArgument('spawn_y', default_value='',
                              description='Override the per-world spawn Y'),
        DeclareLaunchArgument('spawn_z', default_value='',
                              description='Override the per-world spawn Z'),
        DeclareLaunchArgument('spawn_yaw', default_value='',
                              description='Override the per-world spawn yaw'),
        OpaqueFunction(function=resolve),
    ])
