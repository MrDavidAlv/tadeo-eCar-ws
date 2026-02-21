import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch SLAM Toolbox (async mode). Agnostic of sim or real robot."""

    pkg_tadeocar_slam = get_package_share_directory('tadeocar_slam')
    slam_params_file = os.path.join(pkg_tadeocar_slam, 'config', 'slam_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=slam_params_file)

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        DeclareLaunchArgument('params_file', default_value=slam_params_file,
                              description='Full path to SLAM parameters file'),
        slam_toolbox_node
    ])
