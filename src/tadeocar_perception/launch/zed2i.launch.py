#!/usr/bin/env python3
"""The physical ZED 2i, through zed-ros2-wrapper.

Nothing in this workspace depends on this file to run in simulation: Gazebo
renders the camera and simulation.launch.py bridges it under the same topic
names the wrapper uses. This is the swap-in for hardware.

zed_wrapper is not declared in package.xml on purpose. It is not in rosdistro -
it is built from source against the Stereolabs SDK, which needs CUDA - so
declaring it would break `rosdep install` for anyone who only wants the
simulation. The cost of that choice is that this file has to fail with a
sentence a person can act on rather than with an ament lookup error.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    try:
        import os

        from ament_index_python.packages import get_package_share_directory
        wrapper = get_package_share_directory('zed_wrapper')
        source = os.path.join(wrapper, 'launch', 'zed_camera.launch.py')
        available = os.path.exists(source)
    except Exception:
        available = False
        source = None

    if not available:
        return LaunchDescription([LogInfo(msg=(
            'zed_wrapper is not installed, so the physical ZED 2i cannot be '
            'started. Build it from source against the Stereolabs SDK: '
            'https://github.com/stereolabs/zed-ros2-wrapper. The simulated '
            'camera needs none of this - it comes from '
            'tadeocar_gazebo/launch/simulation.launch.py.'))])

    return LaunchDescription([
        DeclareLaunchArgument('camera_model', default_value='zed2i',
                              description='Stereolabs camera model'),
        DeclareLaunchArgument('camera_name', default_value='zed',
                              description='Namespace, which is what makes the '
                                          'wrapper publish under /zed/zed_node '
                                          'exactly as the simulation does'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(source),
            launch_arguments={
                'camera_model': LaunchConfiguration('camera_model'),
                'camera_name': LaunchConfiguration('camera_name'),
                # The wrapper publishes odom -> base_link by default. The EKF
                # owns that transform here, so the wrapper must not.
                'publish_tf': 'false',
                'publish_map_tf': 'false',
            }.items()),
    ])
