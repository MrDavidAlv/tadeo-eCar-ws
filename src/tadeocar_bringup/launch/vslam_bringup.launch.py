#!/usr/bin/env python3
"""Orchestrator: simulation + Visual SLAM (RTAB-Map) with ZED2i stereo camera."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_tadeocar_gazebo = get_package_share_directory('tadeocar_gazebo')
    pkg_tadeocar_vslam = get_package_share_directory('tadeocar_vslam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tadeocar_gazebo, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    vslam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tadeocar_vslam, 'launch', 'vslam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localization mode (no new map nodes)'),
        simulation,
        vslam,
    ])
