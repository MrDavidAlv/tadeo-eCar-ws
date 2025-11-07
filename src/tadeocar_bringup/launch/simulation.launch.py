#!/usr/bin/env python3
"""
TadeoeCar Simulation Launch
Gazebo + Xbox Control + 4WS Kinematics + ros2_control
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch Gazebo simulation with Xbox control"""

    # Package directories
    pkg_description = get_package_share_directory('tadeocar_description')
    pkg_control = get_package_share_directory('tadeocar_control')

    # File paths
    world_file = os.path.join(pkg_description, 'worlds', 'tadeocar.world')
    model_sdf = os.path.join(pkg_description, 'models', 'tadeocar_v1', 'model.sdf')
    control_xacro = os.path.join(pkg_description, 'urdf', 'tadeocar_control.urdf.xacro')
    controllers_file = os.path.join(pkg_control, 'config', 'ros2_controllers.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Set Gazebo model path
    gazebo_model_path = os.path.join(pkg_description, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    # Process xacro to URDF
    import xacro
    robot_desc = xacro.process_file(control_xacro, mappings={'controllers_file': controllers_file}).toxml()

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
        output='screen'
    )

    # Spawn robot (SDF for visualization and physics)
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-file', model_sdf,
             '-entity', 'tadeocar_v1',
             '-x', '0',
             '-y', '0',
             '-z', '0.2'],
        output='screen'
    )

    # Controller Spawners for ros2_control
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    controller_spawners = []
    for wheel in ['front_left', 'front_right', 'rear_left', 'rear_right']:
        steering_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[f'{wheel}_steering_controller'],
            output='screen'
        )
        controller_spawners.append(steering_spawner)

        wheel_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[f'{wheel}_wheel_controller'],
            output='screen'
        )
        controller_spawners.append(wheel_spawner)

    # Joy node (Xbox controller)
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

    # Xbox Control Node
    xbox_control = Node(
        package='tadeocar_control',
        executable='xbox_control',
        name='xbox_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_robot,
        joy_node,
        xbox_control,
        fourws_kinematics,
        joint_state_broadcaster_spawner,
    ] + controller_spawners)
