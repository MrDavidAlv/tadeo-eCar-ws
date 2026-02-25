import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Simulation launch file for TadeoeCar on Gz Sim (Harmonic).
    Launches Gz Sim + ros_gz_bridge + Robot State Publisher + odom_to_tf.
    """

    # Package directories
    pkg_tadeocar_gazebo = get_package_share_directory('tadeocar_gazebo')
    pkg_tadeocar_description = get_package_share_directory('tadeocar_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    world_file = os.path.join(pkg_tadeocar_gazebo, 'worlds', 'factory.world')
    urdf_file = os.path.join(pkg_tadeocar_description, 'urdf', 'tadeocar.urdf')
    sdf_file = os.path.join(pkg_tadeocar_gazebo, 'models', 'tadeocar_v1', 'model.sdf')

    # Set Gz Sim resource path for models and worlds
    gz_models_path = os.path.join(pkg_tadeocar_gazebo, 'models')
    gz_worlds_path = os.path.join(pkg_tadeocar_gazebo, 'worlds')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + gz_models_path + ':' + gz_worlds_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_models_path + ':' + gz_worlds_path

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_file)

    # Gz Sim (server + GUI)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # Spawn robot in Gz Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_file,
            '-name', 'tadeocar',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # ROS <-> Gz Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Odometry (Gz -> ROS2)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # LiDAR (Gz -> ROS2)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Clock (Gz -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint States (Gz -> ROS2)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Joint Commands (ROS2 -> Gz) - 4 steering + 4 wheel
            '/model/tadeocar/joint/front_left_steering_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/tadeocar/joint/front_left_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/tadeocar/joint/front_right_steering_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/tadeocar/joint/front_right_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/tadeocar/joint/rear_left_steering_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/tadeocar/joint/rear_left_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/tadeocar/joint/rear_right_steering_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/tadeocar/joint/rear_right_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            # Cameras (Gz -> ROS2)
            '/camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # Robot state publisher (URDF from tadeocar_description)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

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

    # odom -> base_link TF from /odom topic
    odom_to_tf = Node(
        package='tadeocar_gazebo',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4WS Kinematics Controller (converts /cmd_vel -> individual joint commands)
    fourws_kinematics = Node(
        package='tadeocar_control',
        executable='fourws_kinematics',
        name='fourws_kinematics_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='World file'
        ),
        gz_sim,
        spawn_entity,
        bridge,
        robot_state_publisher,
        odom_to_tf,
        fourws_kinematics,
    ])
