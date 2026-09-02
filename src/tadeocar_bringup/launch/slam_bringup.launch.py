#!/usr/bin/env python3
"""LiDAR SLAM: the simulation, SLAM Toolbox and RViz.

    ros2 launch tadeocar_bringup slam_bringup.launch.py
    ros2 launch tadeocar_bringup slam_bringup.launch.py world:=yard
    ros2 launch tadeocar_bringup slam_bringup.launch.py odom_source:=wheel

Everything about the robot itself comes from tadeocar_gazebo's
simulation.launch.py; this file adds the mapper and a viewer and nothing else.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('tadeocar_gazebo')
    pkg_slam = get_package_share_directory('tadeocar_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = os.path.join(pkg_slam, 'rviz', 'slam.rviz')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'odom_source': LaunchConfiguration('odom_source'),
            # SLAM reads the scan, not the camera. The cloud is for the viewer.
            'publish_points': LaunchConfiguration('publish_points'),
        }.items())

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='factory',
                              description='factory, yard, empty or a path'),
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('odom_source', default_value='ekf',
                              choices=['ekf', 'wheel', 'none']),
        DeclareLaunchArgument('publish_points', default_value='true',
                              choices=['true', 'false']),
        simulation,
        slam,
        rviz,
    ])
