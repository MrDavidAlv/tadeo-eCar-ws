import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Save the current SLAM map to disk."""

    pkg_tadeocar_navigation = get_package_share_directory('tadeocar_navigation')
    default_map_path = os.path.join(pkg_tadeocar_navigation, 'maps', 'mapa')

    map_path = LaunchConfiguration('map_path', default=default_map_path)

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-f', map_path]
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_path', default_value=default_map_path,
                              description='Path (without extension) to save the map'),
        map_saver
    ])
