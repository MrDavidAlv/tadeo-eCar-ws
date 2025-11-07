#!/usr/bin/env python3
"""
Complete simulation launch file for TadeoeCar 4WS4WD
Includes: Gazebo, Robot, 4WS Kinematics Controller, Gazebo Bridge
"""

import os
import launch
import launch.conditions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for complete TadeoeCar simulation"""

    # Package directories
    pkg_description = get_package_share_directory('tadeocar_description')
    pkg_control = get_package_share_directory('tadeocar_control')

    # File paths
    world_file = os.path.join(pkg_description, 'worlds', 'tadeocar.world')
    urdf_file = os.path.join(pkg_description, 'urdf', 'tadeocar_tf.urdf')
    model_file = os.path.join(pkg_description, 'models', 'tadeocar_v1', 'model.sdf')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    # Set Gazebo model path
    gazebo_model_path = os.path.join(pkg_description, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    # Robot State Publisher
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=launch.conditions.IfCondition(gui)
    )

    # Spawn robot
    spawn_robot = ExecuteProcess(
        cmd=['gz', 'model',
             '-f', model_file,
             '-m', 'tadeocar_v1',
             '-x', '0',
             '-y', '0',
             '-z', '0.5'],
        output='screen'
    )

    # 4WS Kinematics Controller
    fourws_kinematics = Node(
        package='tadeocar_control',
        executable='fourws_kinematics',
        name='fourws_kinematics_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Gazebo Effort Bridge
    gazebo_bridge = Node(
        package='tadeocar_control',
        executable='gazebo_effort_bridge',
        name='gazebo_effort_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_robot,
        fourws_kinematics,
        gazebo_bridge
    ])
