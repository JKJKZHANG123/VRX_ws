from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config = PathJoinSubstitution(
        [FindPackageShare('usv_localization'), 'config', 'ekf.yaml'])
    navsat_config = PathJoinSubstitution(
        [FindPackageShare('usv_localization'), 'config', 'navsat.yaml'])

    # Covariance injector: both absolute-pose sources (Point-LIO odom in mapping
    # mode, and the gz NavSatFix) publish ZERO covariance, which makes the EKF
    # update step singular -> NaN. This relay stamps a finite diagonal onto both
    # before they reach the filter. Zero intrusion into Point-LIO / the sim.
    cov_injector = Node(
        package='usv_localization',
        executable='covariance_injector',
        name='covariance_injector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Frame reconciliation (see covariance_injector.py spawn_yaw_offset):
    # Point-LIO's camera_init is defined by the boat's spawn heading (the
    # gz IMU is ENU-absolute, and Point-LIO gravity-aligns -> camera_init keeps
    # yaw = spawn yaw 1.0), while the EKF odom frame is ENU-anchored by GPS.
    # camera_init is therefore rotated +spawn_yaw from odom/ENU, and the static
    # TF odom->camera_init carries +spawn_yaw, so /usv/costmap_cloud (frame
    # camera_init) and LIO TF land on the correct ENU positions. Keep this in
    # sync with the injector param (same +spawn_yaw) and the world spawn yaw
    # (vrx_gz/launch/competition.launch.py).
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='1.0')
    odom_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_camera_init_bridge',
        arguments=['0', '0', '0',
                   PythonExpression(["float('", spawn_yaw, "')"]),
                   '0', '0',
                   'odom', 'camera_init'],
        parameters=[{'use_sim_time': True}],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config],
        remappings=[
            ('imu', '/wamv/sensors/imu/imu/data'),
            ('gps/fix', '/wamv/sensors/gps/gps/fix_cov'),   # covariance-injected
            ('odometry/filtered', '/odometry/filtered'),
            # output: /odometry/gps (consumed by ekf as odom1)
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_yaw', default_value='1.0',
            description='Spawn yaw (rad) of the WAM-V in the ENU world; '
                        'odom->camera_init is rotated by -spawn_yaw'),
        cov_injector,
        odom_bridge,
        ekf,
        navsat,
    ])
