#!/usr/bin/env python3
"""Autonomous navigation: the simulation, Nav2 against a known map, and RViz.

    ros2 launch tadeocar_bringup navigation_bringup.launch.py
    ros2 launch tadeocar_bringup navigation_bringup.launch.py world:=yard

The map follows the world unless `map` names one explicitly, so the two cannot
drift apart by accident. Which one was picked is logged at startup.

Nav2's velocity output is remapped to cmd_vel_nav so twist_mux arbitrates it
against the joystick and the web interface rather than fighting them. Priority
order is in tadeocar_control/config/twist_mux.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
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
    def navigation(context, *_args, **_kwargs):
        """Resolve the map in the parent context, so it can follow the world.

        ``map`` used to default to 'factory' on its own, which meant that
        ``world:=yard`` drove the yard while Nav2 planned against the factory:
        a 22.5 x 17.5 m map for a 32.4 x 22.4 m world, with the building's
        walls laid over open tarmac. Nothing errored. The global costmap came
        up the wrong size, the planner reported "failed to create plan" for
        goals standing in open space, and the only way to see why was to
        compare the costmap's dimensions against the world's.

        The default is empty and means "the same name as the world". An
        explicit map still wins, so the two can still be mismatched on
        purpose - localising against a SLAM-built map, say.
        """
        world_name = LaunchConfiguration('world').perform(context)
        requested = LaunchConfiguration('map').perform(context).strip()
        chosen = requested or world_name
        return [
            LogInfo(msg=f'navigation: world={world_name} map={chosen}'),
            GroupAction(actions=[
                SetRemap('/cmd_vel', '/cmd_vel_nav'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_navigation, 'launch',
                                     'navigation.launch.py')),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'map': chosen,
                        # AMCL's initial pose has to match where the robot
                        # spawns, which is a property of the world rather
                        # than of the map.
                        'world': world_name,
                    }.items()),
            ]),
        ]

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='factory',
                              description='factory, yard, empty or a path'),
        DeclareLaunchArgument('map', default_value='',
                              description='World name for the ground truth '
                                          'map, or a path to a map yaml. '
                                          'Empty means: follow the world.'),
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('odom_source', default_value='ekf',
                              choices=['ekf', 'wheel', 'none']),
        DeclareLaunchArgument('publish_points', default_value='true',
                              choices=['true', 'false']),
        simulation,
        OpaqueFunction(function=navigation),
        rviz,
    ])
