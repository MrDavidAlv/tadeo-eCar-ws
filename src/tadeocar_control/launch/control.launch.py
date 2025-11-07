from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('tadeocar_control')
    web_dir = os.path.join(pkg_dir, 'web')

    return LaunchDescription([
        # Web control node
        Node(
            package='tadeocar_control',
            executable='web_control',
            name='web_control_node',
            output='screen'
        ),

        # HTTP server for web interface
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8080'],
            cwd=web_dir,
            output='screen',
            shell=False
        )
    ])
