#!/usr/bin/env python3
"""robot_localization, agnostic of simulation or hardware.

Which configuration it loads is the caller's choice: simulation.launch.py
passes config/ekf_sim.yaml, the real robot's bringup passes config/ekf_real.yaml.
The two differ only in where absolute position comes from - see the header of
either file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tadeocar_perception')
    default_config = os.path.join(pkg, 'config', 'ekf_sim.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    publish_tf = LaunchConfiguration('publish_tf')

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file, {
            'use_sim_time': use_sim_time,
            # Overrides the value in the YAML. A caller that wants the filter's
            # estimate published on /odometry/filtered without it owning
            # odom -> base_footprint - because visual odometry is publishing
            # that transform instead - sets this false and still gets a fused
            # pose to compare against.
            'publish_tf': publish_tf,
        }],
        remappings=[('/odometry/filtered', '/odometry/filtered')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='EKF parameter file'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              choices=['true', 'false'],
                              description='Publish odom -> base_footprint'),
        ekf,
    ])
