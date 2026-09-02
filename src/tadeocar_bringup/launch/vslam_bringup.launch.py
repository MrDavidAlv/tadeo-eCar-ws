#!/usr/bin/env python3
"""Visual SLAM: the simulation, RTAB-Map on the ZED 2i, and RViz.

    ros2 launch tadeocar_bringup vslam_bringup.launch.py
    ros2 launch tadeocar_bringup vslam_bringup.launch.py world:=yard
    ros2 launch tadeocar_bringup vslam_bringup.launch.py odom_source:=ekf
    ros2 launch tadeocar_bringup vslam_bringup.launch.py rtabmap_viz:=true

RTAB-Map builds the map, closes loops and publishes the 3D cloud. What it maps
on top of is the odom_source argument, and the default is deliberate:

  ekf     (default) the fused wheel-and-IMU pose owns odom -> base_footprint
          and RTAB-Map corrects it with map -> odom. This is an ordinary RGB-D
          SLAM setup and it is the one that works here.

  visual  rgbd_odometry owns the transform and nothing but the camera says
          where the robot is. Honest, and measurably not good enough in this
          world yet: see docs/visual-slam.md for the numbers and why.

Either way the EKF runs, so /odometry/filtered is always there to compare
against.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('tadeocar_gazebo')
    pkg_vslam = get_package_share_directory('tadeocar_vslam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_source = LaunchConfiguration('odom_source').perform(context)

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            # With visual odometry owning the transform the simulation must not
            # publish it. The EKF still runs and still publishes
            # /odometry/filtered, so the two estimates stay comparable.
            'odom_source': 'none' if odom_source == 'visual' else 'ekf',
            # RTAB-Map reads the depth image directly and does not need
            # this, but the live cloud is the thing worth watching in RViz:
            # RTAB-Map's own /cloud_map is the accumulated result, and only
            # the reprojected cloud shows what the camera is seeing right
            # now. It costs about a fifth of a core; pass publish_points:=false
            # to get it back.
            'publish_points': LaunchConfiguration('publish_points'),
        }.items())

    vslam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vslam, 'launch', 'vslam.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_source': odom_source,
            'localization': LaunchConfiguration('localization'),
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
        }.items())

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_vslam, 'rviz', 'vslam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return [simulation, vslam, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='factory',
                              description='factory, yard, empty or a path'),
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('odom_source', default_value='ekf',
                              choices=['visual', 'ekf'],
                              description='Whose pose RTAB-Map maps on top of'),
        DeclareLaunchArgument('localization', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('publish_points', default_value='true',
                              choices=['true', 'false'],
                              description='Reproject the ZED depth image into '
                                          'a live point cloud for RViz'),
        OpaqueFunction(function=resolve),
    ])
