#!/usr/bin/env python3
"""
Complete simulation with Xbox controller
Includes: Gazebo, Robot, Xbox Control, 4WS Kinematics
"""

import os
import launch
import launch.conditions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for TadeoeCar simulation with Xbox control"""

    # Package directories
    pkg_description = get_package_share_directory('tadeocar_description')
    pkg_control = get_package_share_directory('tadeocar_control')

    # File paths
    world_file = os.path.join(pkg_description, 'worlds', 'tadeocar.world')
    urdf_file = os.path.join(pkg_description, 'urdf', 'tadeocar_control.urdf.xacro')
    model_file = os.path.join(pkg_description, 'models', 'tadeocar_v1', 'model.sdf')
    controllers_file = os.path.join(pkg_control, 'config', 'ros2_controllers.yaml')

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

    # Robot State Publisher - Process xacro to URDF
    import xacro
    robot_desc = xacro.process_file(urdf_file, mappings={'controllers_file': controllers_file}).toxml()

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

    # Spawn robot using spawn_entity.py (ROS2 way)
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-file', model_file,
             '-entity', 'tadeocar_v1',
             '-x', '0',
             '-y', '0',
             '-z', '0.2'],
        output='screen'
    )

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

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Steering and wheel controller spawners
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

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_robot,
        joy_node,
        xbox_control,
        fourws_kinematics,
        joint_state_broadcaster_spawner
    ] + controller_spawners)
