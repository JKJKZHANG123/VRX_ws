"""
GPS anchoring for Point-LIO — navsat_transform_node ONLY (no EKF).

Publishes utm → camera_init transform, anchoring the LIO world frame to GPS.
This prevents long-term drift on open water where LIO degrades (lack of
geometric features). Nav2 still uses Point-LIO's /aft_mapped_to_init as the
odom source; this just keeps the world frame globally consistent.

Launch after Point-LIO is running and publishing /aft_mapped_to_init.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('usv_localization')
    navsat_params = os.path.join(pkg_share, 'config', 'navsat.yaml')

    return LaunchDescription([
        # GPS/UTM fusion: anchors camera_init (LIO world) to UTM coordinates.
        # use_odometry_yaw: true — trust LIO heading over magnetometer.
        # zero_altitude: true — trust LIO z over GPS altitude.
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_params],
            remappings=[
                ('gps/fix', '/wamv/sensors/gps/gps/fix'),
                ('odometry/filtered', '/aft_mapped_to_init'),
            ],
        ),
    ])
