#!/usr/bin/env python3
"""RGB-D visual SLAM with RTAB-Map, agnostic of simulation or hardware.

Two nodes, and the split matters:

  rgbd_odometry  estimates the camera's motion frame to frame from the images
                 themselves. This is the part that makes it visual SLAM rather
                 than a mapper riding on someone else's odometry.
  rtabmap        builds the map, closes loops and corrects the trajectory.

``odom_source`` decides which of them owns odom -> base_footprint, because a
frame has exactly one parent:

  visual  rgbd_odometry publishes it, and the EKF is not started. This is the
          honest visual SLAM demo: nothing but the camera says where the robot
          is between loop closures.
  ekf     the EKF publishes it and RTAB-Map maps on top of the fused pose.
          Better maps, but it no longer demonstrates visual odometry.

Both consume the same topics the real ZED 2i publishes, so nothing here
changes when the simulated camera is swapped for the physical one.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

RGB = '/zed/zed_node/rgb/image_rect_color'
DEPTH = '/zed/zed_node/depth/depth_registered'
INFO = '/zed/zed_node/rgb/camera_info'


def str_params(pairs):
    """RTAB-Map's own parameters are all strings.

    launch_ros infers a parameter's type from its Python value, so 'false'
    arrives as a bool and the node aborts with InvalidParameterTypeException
    before it has published anything. Wrapping each value in a ParameterValue
    with value_type=str is what keeps them strings.
    """
    return {k: ParameterValue(v, value_type=str) for k, v in pairs.items()}


def resolve(context, *args, **kwargs):
    pkg = get_package_share_directory('tadeocar_vslam')
    params_file = os.path.join(pkg, 'config', 'rtabmap_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_source = LaunchConfiguration('odom_source').perform(context)
    localization = LaunchConfiguration('localization').perform(context) == 'true'
    database = os.path.expanduser(
        LaunchConfiguration('database_path').perform(context))

    common = {'use_sim_time': use_sim_time,
              'frame_id': 'base_footprint',
              'odom_frame_id': 'odom',
              'map_frame_id': 'map',
              # RGB-D means subscribe_depth. subscribe_rgb is the RGB-ONLY
              # mode, and setting both leaves rtabmap picking one of two
              # incompatible input configurations at startup.
              'subscribe_depth': True,
              'subscribe_rgb': False,
              'subscribe_scan': False,
              'approx_sync': True,
              'queue_size': 30,
              'qos_image': 1,
              'qos_camera_info': 1}

    remaps = [('rgb/image', RGB), ('depth/image', DEPTH),
              ('rgb/camera_info', INFO)]

    nodes = []

    # Visual odometry. Only started when it is the one publishing the
    # transform; with odom_source ekf the fused pose already fills that role
    # and running both would put two parents on one frame.
    if odom_source == 'visual':
        nodes.append(Node(
            package='rtabmap_odom', executable='rgbd_odometry',
            name='rgbd_odometry', output='screen',
            parameters=[params_file, common, str_params({
                'Odom/Strategy': '0',            # frame-to-map
                'Odom/ResetCountdown': '1',
                'Vis/MinInliers': '10',
                'Vis/EstimationType': '1',
                'GFTT/MinDistance': '7',
                'GFTT/QualityLevel': '0.0003',
            }), {'publish_tf': True, 'publish_null_when_lost': False}],
            remappings=remaps + [('odom', '/odom_visual')]))

    rtabmap_params = str_params({
        'Mem/IncrementalMemory': 'false' if localization else 'true',
        'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
    })

    nodes.append(Node(
        package='rtabmap_slam', executable='rtabmap',
        name='rtabmap', output='screen',
        parameters=[params_file, common, rtabmap_params,
                    {'database_path': database,
                     # RTAB-Map corrects map -> odom. Whoever owns
                     # odom -> base_footprint keeps it.
                     'publish_tf': True}],
        remappings=remaps + [
            ('odom', '/odom_visual' if odom_source == 'visual'
                     else '/odometry/filtered')],
        arguments=[] if localization else ['--delete_db_on_start']))

    nodes.append(Node(
        package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz',
        output='screen',
        parameters=[params_file, common,
                    {'subscribe_odom_info': odom_source == 'visual'}],
        remappings=remaps + [
            ('odom', '/odom_visual' if odom_source == 'visual'
                     else '/odometry/filtered')],
        condition=IfCondition(LaunchConfiguration('rtabmap_viz'))))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'odom_source', default_value='ekf', choices=['visual', 'ekf'],
            description='visual runs rgbd_odometry and lets it own '
                        'odom -> base_footprint; ekf maps on the fused pose'),
        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Reuse the existing database instead of building a '
                        'new map'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap_tadeocar.db',
            description='Where the map database lives. Explicit because '
                        'RTAB-Map otherwise shares ~/.ros/rtabmap.db with '
                        'every other project on the machine'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              choices=['true', 'false'],
                              description="RTAB-Map's own window, showing "
                                          'feature matches frame by frame'),
        OpaqueFunction(function=resolve),
    ])
