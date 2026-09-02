#!/usr/bin/env python3
"""The demo: Gazebo, RViz and a way to drive, in one command.

    ros2 launch tadeocar_bringup demo.launch.py                 # the yard
    ros2 launch tadeocar_bringup demo.launch.py world:=factory
    ros2 launch tadeocar_bringup demo.launch.py use_ekf:=false  # the contrast

The yard is the default because it is the world where the difference between
the two estimators is visible rather than merely measurable. The robot starts
at (-9.0, 4.5) facing the north ramp, so the first thing anyone does - hold
forward - is exactly what the demo exists to show: drive up, and watch the
wheel odometry arrow stay flat on the ground while the fused pose climbs with
the robot.

use_ekf:=false is worth running once. It hands odom -> base_footprint back to
dead reckoning, and on the platform the robot's own idea of where it is sits
0.35 m below where it obviously is.

The web interface comes up on http://localhost:8080 - keyboard, joystick and
sliders, publishing to cmd_vel_web. An Xbox pad works too:
ros2 launch tadeocar_control xbox_control.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('tadeocar_gazebo')
    pkg_slam = get_package_share_directory('tadeocar_slam')
    pkg_control = get_package_share_directory('tadeocar_control')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf = LaunchConfiguration('use_ekf').perform(context) == 'true'

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': LaunchConfiguration('world'),
            'headless': 'false',
            'odom_source': 'ekf' if use_ekf else 'wheel',
            'publish_points': 'true',
        }.items())

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_slam, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}])

    web_control = Node(
        package='tadeocar_control', executable='web_control',
        name='web_control_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('web')))

    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8080'],
        cwd=os.path.join(pkg_control, 'web'), output='screen',
        condition=IfCondition(LaunchConfiguration('web')))

    return [simulation, rviz, web_control, web_server]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='yard',
                              description='yard, factory, empty or a path'),
        DeclareLaunchArgument(
            'use_ekf', default_value='true', choices=['true', 'false'],
            description='Fuse the ZED 2i IMU with the wheels. False falls back '
                        'to dead reckoning, which assumes the ground is flat'),
        DeclareLaunchArgument('web', default_value='true',
                              choices=['true', 'false'],
                              description='Serve the web control interface on '
                                          'http://localhost:8080'),
        OpaqueFunction(function=resolve),
    ])
