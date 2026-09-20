# Copyright 2026 jkjkzhang

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('usv_cloud_filter'),
         'config', 'cloud_filter_params.yaml'])

    obstacle_filter = Node(
        package='usv_cloud_filter',
        executable='obstacle_filter',
        name='obstacle_filter',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([obstacle_filter])
