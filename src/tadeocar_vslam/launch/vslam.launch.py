import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch RTAB-Map for Visual SLAM with ZED2i stereo camera.
    Uses Gazebo odometry as input; stereo images for loop closure and mapping.
    Topic names match real ZED SDK for sim-to-real compatibility.
    """

    pkg_vslam = get_package_share_directory('tadeocar_vslam')
    params_file = os.path.join(pkg_vslam, 'config', 'rtabmap_params.yaml')
    rviz_file = os.path.join(pkg_vslam, 'rviz', 'vslam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    localization = LaunchConfiguration('localization', default='false')

    # Database path for RTAB-Map (explicit to avoid stale state)
    db_path = os.path.expanduser('~/.ros/rtabmap_tadeocar.db')

    common_remappings = [
        ('left/image_rect', '/zed/zed_node/left/image_rect_color'),
        ('right/image_rect', '/zed/zed_node/right/image_rect_color'),
        ('left/camera_info', '/zed/zed_node/left/camera_info'),
        ('right/camera_info', '/zed/zed_node/right/camera_info'),
        ('odom', '/odom'),
    ]

    common_params = {
        'use_sim_time': use_sim_time,
        'database_path': db_path,
    }

    # Mapping mode: IncrementalMemory=true, delete DB on start
    rtabmap_mapping = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            params_file,
            {**common_params, 'Mem/IncrementalMemory': 'true'},
        ],
        remappings=common_remappings,
        arguments=['--delete_db_on_start'],
    )

    # Localization mode: IncrementalMemory=false, keep existing DB
    rtabmap_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            params_file,
            {**common_params, 'Mem/IncrementalMemory': 'false'},
        ],
        remappings=common_remappings,
    )

    # RViz for visual SLAM visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_vslam',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Set true for localization mode (no new nodes in map)'),
        rtabmap_mapping,
        rtabmap_localization,
        rviz_node,
    ])
