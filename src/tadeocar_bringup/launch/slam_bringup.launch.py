#!/usr/bin/env python3
"""
SLAM bringup for TadeoeCar
Complete SLAM system with Gazebo, Xbox control, and mapping
"""

import os
import launch
import launch.conditions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch complete SLAM system"""

    # Package directories
    pkg_bringup = get_package_share_directory('tadeocar_bringup')
    pkg_navigation = get_package_share_directory('tadeocar_navigation')

    # File paths
    slam_params_file = os.path.join(pkg_navigation, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_navigation, 'config', 'slam.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    # Include control bringup (Gazebo + Xbox + Kinematics)
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'control_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui
        }.items()
    )

    # SLAM Toolbox (Async)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        control_launch,
        slam_toolbox_node,
        rviz_node
    ])
