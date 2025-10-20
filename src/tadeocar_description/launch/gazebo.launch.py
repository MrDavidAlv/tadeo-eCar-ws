import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('tadeocar_description')
    world_file = os.path.join(pkg_dir, 'worlds', 'tadeocar.world')
    model_path = os.path.join(pkg_dir, 'models')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'tadeocar_tf.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    robot_description = Command(['cat ', urdf_file])

    gazebo_env = {
        'GAZEBO_MODEL_PATH': model_path
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env=gazebo_env
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui),
        additional_env=gazebo_env
    )

    spawn_entity = ExecuteProcess(
        cmd=['gz', 'model', '-f', os.path.join(model_path, 'tadeocar_v1', 'model.sdf'), '-m', 'tadeocar_v1', '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_entity
    ])
