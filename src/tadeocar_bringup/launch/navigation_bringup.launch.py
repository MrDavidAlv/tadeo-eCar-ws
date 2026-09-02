#!/usr/bin/env python3
"""Autonomous navigation: the simulation, Nav2 against a known map, and RViz.

    ros2 launch tadeocar_bringup navigation_bringup.launch.py
    ros2 launch tadeocar_bringup navigation_bringup.launch.py world:=yard

Nav2's velocity output is remapped to cmd_vel_nav so twist_mux arbitrates it
against the joystick and the web interface rather than fighting them. Priority
order is in tadeocar_control/config/twist_mux.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('tadeocar_gazebo')
    pkg_navigation = get_package_share_directory('tadeocar_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    rviz_config = os.path.join(pkg_navigation, 'rviz', 'navigation.rviz')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'headless': LaunchConfiguration('headless'),
            'odom_source': LaunchConfiguration('odom_source'),
            'publish_points': LaunchConfiguration('publish_points'),
        }.items())

    # There is no static map -> odom publisher here, and that is the fix rather
    # than an omission. This file used to start one "until AMCL initialises",
    # but AMCL publishes that transform from the moment it comes up, so the two
    # were writing the same transform forever: the robot's map pose flickered
    # between the localised estimate and the origin, several times a second,
    # and every costmap saw both.
    navigation = GroupAction(actions=[
        SetRemap('/cmd_vel', '/cmd_vel_nav'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': LaunchConfiguration('map'),
                # AMCL's initial pose has to match where the robot spawns,
                # which is a property of the world rather than of the map.
                'world': world,
            }.items()),
    ])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='factory',
                              description='factory, yard, empty or a path'),
        DeclareLaunchArgument('map', default_value='factory',
                              description='World name for the ground truth '
                                          'map, or a path to a map yaml'),
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('odom_source', default_value='ekf',
                              choices=['ekf', 'wheel', 'none']),
        DeclareLaunchArgument('publish_points', default_value='true',
                              choices=['true', 'false']),
        simulation,
        navigation,
        rviz,
    ])
