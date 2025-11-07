#!/usr/bin/env python3
"""
Save map launch file
Saves current SLAM map to file
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """Save current map"""

    # Launch arguments
    map_name = LaunchConfiguration('map_name', default='my_map')

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='my_map',
        description='Name for the saved map file'
    )

    # Save map
    save_map = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', map_name],
        output='screen'
    )

    return LaunchDescription([
        declare_map_name,
        save_map
    ])
