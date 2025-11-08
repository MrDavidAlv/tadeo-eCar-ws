#!/usr/bin/env python3
"""
TadeoeCar Navigation
Launches Gazebo, Xbox control, and Navigation2 with RViz2
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_description = get_package_share_directory('tadeocar_description')
    pkg_navigation = get_package_share_directory('tadeocar_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_navigation, 'maps', 'mapa.yaml')
    rviz_config = os.path.join(pkg_navigation, 'config', 'navigation.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    map_file = LaunchConfiguration('map')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui
        }.items()
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    xbox_control = Node(
        package='tadeocar_control',
        executable='xbox_control',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    fourws_kinematics = Node(
        package='tadeocar_control',
        executable='fourws_kinematics',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'map': map_file
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file'
        ),
        gazebo_launch,
        joy_node,
        xbox_control,
        fourws_kinematics,
        nav2_launch,
        rviz
    ])
