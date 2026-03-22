#!/usr/bin/env python3
"""Orchestrator: simulation + fourws_kinematics + twist_mux + Nav2 + RViz."""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_tadeocar_gazebo = get_package_share_directory('tadeocar_gazebo')
    pkg_tadeocar_navigation = get_package_share_directory('tadeocar_navigation')

    rviz_config = os.path.join(pkg_tadeocar_navigation, 'rviz', 'navigation.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Simulation (includes gz_sim + bridge + robot_state_pub + fourws_kinematics + twist_mux)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tadeocar_gazebo, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Nav2 with cmd_vel remapped to cmd_vel_nav so twist_mux handles priority
    navigation = GroupAction(
        actions=[
            SetRemap('/cmd_vel', '/cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_tadeocar_navigation, 'launch', 'navigation.launch.py'
                    )
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
        ]
    )

    # Static TF map->odom (until AMCL initializes)
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        simulation,
        static_transform_publisher,
        navigation,
        rviz,
    ])
