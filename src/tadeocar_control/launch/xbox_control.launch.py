#!/usr/bin/env python3
"""Xbox pad teleoperation: joy_node plus the mapping node.

    ros2 launch tadeocar_control xbox_control.launch.py

fourws_kinematics_node and twist_mux come from simulation.launch.py, so this
runs alongside any of the bringups. The pad publishes on /cmd_vel_joy, which
twist_mux ranks above the web interface and Nav2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node', output='screen',
        parameters=[{
            # ROS 2's joy takes device_id, an index into the enumerated pads.
            # This file used to pass 'dev': '/dev/input/js0', which is the ROS 1
            # parameter; it was accepted, ignored, and worked only because the
            # default index already pointed at the same pad.
            'device_id': LaunchConfiguration('device_id'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'use_sim_time': use_sim_time,
        }])

    xbox_control_node = Node(
        package='tadeocar_control', executable='xbox_control',
        name='xbox_control_node', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('device_id', default_value='0',
                              description='Index of the pad, as listed by '
                                          'ros2 run joy joy_enumerate_devices'),
        # The ceilings fourws_kinematics_node clamps to. Asking for more only
        # moves the point where the stick stops doing anything.
        DeclareLaunchArgument('max_linear_speed', default_value='1.0'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.0'),
        joy_node,
        xbox_control_node,
    ])
