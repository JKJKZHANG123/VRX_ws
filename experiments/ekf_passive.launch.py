"""Passive external EKF chain for controlled A/B evaluation.

It publishes /odometry/filtered but does not publish odom->base_link, so it
cannot interfere with the already validated Point-LIO/Nav2 TF path. Nav2 is
left on Point-LIO during these first experiments; this launch is for measuring
whether the EKF estimate itself is better or worse.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config = PathJoinSubstitution([
        FindPackageShare('usv_localization'), 'config', 'ekf.yaml'])
    navsat_config = PathJoinSubstitution([
        FindPackageShare('usv_localization'), 'config', 'navsat.yaml'])
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    return LaunchDescription([
        DeclareLaunchArgument('spawn_yaw', default_value='1.0'),
        Node(
            package='usv_localization', executable='covariance_injector',
            name='covariance_injector', output='screen',
            parameters=[{'use_sim_time': True}]),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='ekf_odom_camera_init_bridge',
            arguments=['0', '0', '0',
                       PythonExpression(["float('", spawn_yaw, "')"]),
                       '0', '0', 'odom', 'camera_init'],
            parameters=[{'use_sim_time': True}]),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', output='screen',
            parameters=[ekf_config, {'publish_tf': False}]),
        Node(
            package='robot_localization', executable='navsat_transform_node',
            name='navsat_transform_node', output='screen',
            parameters=[navsat_config],
            remappings=[
                ('imu', '/wamv/sensors/imu/imu/data'),
                ('gps/fix', '/wamv/sensors/gps/gps/fix_cov'),
                ('odometry/filtered', '/odometry/filtered'),
            ]),
    ])
