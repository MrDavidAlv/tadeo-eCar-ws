#!/usr/bin/env python3
"""
Launch file for 4WS4WD kinematics control system
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with 4WS kinematics node"""

    # 4WS Kinematics Controller
    fourws_kinematics_node = Node(
        package='tadeocar_control',
        executable='fourws_kinematics',
        name='fourws_kinematics_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        fourws_kinematics_node
    ])
