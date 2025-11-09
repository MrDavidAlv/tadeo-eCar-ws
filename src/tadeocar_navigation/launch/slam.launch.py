#!/usr/bin/env python3
"""
SLAM launch file for TadeoeCar with Xbox control
Includes: Gazebo, Robot, Xbox Control, SLAM Toolbox, RViz
"""

import os
import launch
import launch.conditions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for TadeoeCar SLAM"""

    # Package directories
    pkg_description = get_package_share_directory('tadeocar_description')
    pkg_control = get_package_share_directory('tadeocar_control')
    pkg_navigation = get_package_share_directory('tadeocar_navigation')

    # File paths
    world_file = os.path.join(pkg_description, 'worlds', 'tadeocar.world')
    urdf_file = os.path.join(pkg_description, 'urdf', 'tadeocar_tf.urdf')
    model_file = os.path.join(pkg_description, 'models', 'tadeocar_v1', 'model.sdf')
    slam_params_file = os.path.join(pkg_navigation, 'config', 'slam_params.yaml')

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

    # 4WS Odometry Node (calculates odometry from all 4 wheels) - DISABLED TEMPORARILY
    # TODO: Debug and re-enable once working properly
    # fourws_odometry = Node(
    #     package='tadeocar_control',
    #     executable='fourws_odometry',
    #     name='fourws_odometry_node',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }]
    # )

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

    # SLAM Toolbox (Async)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz2
    rviz_config = os.path.join(pkg_navigation, 'config', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

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
        gazebo_bridge,
        slam_toolbox_node,
        rviz_node
    ])
