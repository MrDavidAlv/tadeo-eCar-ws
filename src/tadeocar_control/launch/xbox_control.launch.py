#!/usr/bin/env python3
"""
Launch file for Xbox controller.
Launches joy_node + xbox_control_node only.
fourws_kinematics and twist_mux are launched by simulation.launch.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Joy node (reads from /dev/input/js0)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    # Xbox Control Node (maps joy -> /cmd_vel_joy + /robot_mode)
    xbox_control_node = Node(
        package='tadeocar_control',
        executable='xbox_control',
        name='xbox_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        joy_node,
        xbox_control_node,
    ])
