#!/usr/bin/env python3
"""Nav2, agnostic of simulation or hardware.

The map argument takes a short world name - factory, yard - and resolves it to
that world's generated ground truth, or a path to any map yaml. Ground truth is
the right default for a demo because it is exactly the building: judging the
planner against a map SLAM happened to produce mixes two questions.

    ros2 launch tadeocar_navigation navigation.launch.py
    ros2 launch tadeocar_navigation navigation.launch.py map:=yard
    ros2 launch tadeocar_navigation navigation.launch.py map:=/path/to/my_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def resolve(context, *args, **kwargs):
    pkg_navigation = get_package_share_directory('tadeocar_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    given = LaunchConfiguration('map').perform(context)
    if given.endswith(('.yaml', '.yml')) or os.path.sep in given:
        map_yaml = given
    else:
        map_yaml = os.path.join(pkg_navigation, 'maps',
                                f'{given}_ground_truth.yaml')
    if not os.path.exists(map_yaml):
        raise RuntimeError(
            f'map "{given}" not found at {map_yaml}. Pass a world name whose '
            f'generator has been run, or the path to a map yaml.')

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('params_file'),
            'map': map_yaml,
        }.items())]


def generate_launch_description():
    pkg_navigation = get_package_share_directory('tadeocar_navigation')
    nav2_params = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=nav2_params),
        DeclareLaunchArgument(
            'map', default_value='factory',
            description='A world name, or a path to a map yaml'),
        OpaqueFunction(function=resolve),
    ])
