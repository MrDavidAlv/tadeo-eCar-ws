#!/usr/bin/env python3
"""Nav2, agnostic of simulation or hardware.

The map argument takes a short world name - factory, yard - and resolves it to
that world's generated ground truth, or a path to any map yaml. Ground truth is
the right default for a demo because it is exactly the building: judging the
planner against a map SLAM happened to produce mixes two questions.

    ros2 launch tadeocar_navigation navigation.launch.py
    ros2 launch tadeocar_navigation navigation.launch.py map:=yard
    ros2 launch tadeocar_navigation navigation.launch.py map:=/path/to/my_map.yaml
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Where AMCL should assume the robot is at startup, per world. It has to match
# tadeocar_gazebo's spawn table: AMCL's initial_pose was hardcoded to the
# origin, which is right for the factory and 10 m wrong for the yard, where the
# robot spawns at the foot of the north ramp. A particle filter started 10 m
# from the robot does not converge, it relocalises - if it can.
INITIAL_POSE = {
    'factory': (0.0, 0.0, 0.0),
    'yard': (-9.0, 4.5, 0.0),
    'empty': (0.0, 0.0, 0.0),
}


def resolve(context, *args, **kwargs):
    pkg_navigation = get_package_share_directory('tadeocar_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    given = LaunchConfiguration('map').perform(context)
    if given.endswith(('.yaml', '.yml')) or os.path.sep in given:
        map_yaml = given
    else:
        map_yaml = os.path.join(pkg_navigation, 'maps',
                                f'{given}_ground_truth.yaml')
    if not os.path.exists(map_yaml):
        raise RuntimeError(
            f'map "{given}" not found at {map_yaml}. Pass a world name whose '
            f'generator has been run, or the path to a map yaml.')

    # Write the initial pose into a copy of the parameter file.
    #
    # nav2_common's RewrittenYaml was the obvious tool and does not work here:
    # it matches rewrite keys by name anywhere in the tree, so it cannot
    # address amcl's nested initial_pose.x without also rewriting every other
    # key called x. Editing the file is explicit and does what it says.
    world = LaunchConfiguration('world').perform(context) or given
    x, y, yaw = INITIAL_POSE.get(world, INITIAL_POSE['factory'])
    source = LaunchConfiguration('params_file').perform(context)
    with open(source) as f:
        config = yaml.safe_load(f)
    amcl = config.get('amcl', {}).get('ros__parameters')
    if amcl is not None:
        amcl['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
    handle = tempfile.NamedTemporaryFile(
        mode='w', prefix='tadeocar_nav2_', suffix='.yaml', delete=False)
    yaml.safe_dump(config, handle)
    handle.close()
    params = handle.name

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': params,
            'map': map_yaml,
        }.items())]


def generate_launch_description():
    pkg_navigation = get_package_share_directory('tadeocar_navigation')
    nav2_params = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=nav2_params),
        DeclareLaunchArgument(
            'map', default_value='factory',
            description='A world name, or a path to a map yaml'),
        DeclareLaunchArgument(
            'world', default_value='',
            description="World name, used to place AMCL's initial pose. "
                        'Defaults to whatever `map` names'),
        OpaqueFunction(function=resolve),
    ])
