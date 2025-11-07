#!/usr/bin/env python3
"""
NAV2 Navigation Launch for TadeoeCar
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch NAV2 navigation stack"""

    # Package directories
    pkg_navigation = get_package_share_directory('tadeocar_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default='')
    params_file = LaunchConfiguration('params_file',
                                       default=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'))

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'),
        description='Full path to nav2 parameters file'
    )

    # NAV2 Bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        nav2_launch
    ])
