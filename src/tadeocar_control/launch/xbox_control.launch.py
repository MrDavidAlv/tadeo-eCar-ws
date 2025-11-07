#!/usr/bin/env python3
"""
Launch file for Xbox controller with 4WS kinematics
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description with Xbox control"""

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

    # Xbox Control Node (maps joy to cmd_vel)
    xbox_control_node = Node(
        package='tadeocar_control',
        executable='xbox_control',
        name='xbox_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # 4WS Kinematics Controller
    fourws_kinematics_node = Node(
        package='tadeocar_control',
        executable='fourws_kinematics',
        name='fourws_kinematics_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        joy_node,
        xbox_control_node,
        fourws_kinematics_node
    ])
