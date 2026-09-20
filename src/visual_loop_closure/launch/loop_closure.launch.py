"""
启动视觉回环检测 + 位姿图优化节点
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('visual_loop_closure')
    config = os.path.join(pkg_dir, 'config', 'loop_closure_params.yaml')

    node = Node(
        package='visual_loop_closure',
        executable='visual_slam_node',
        name='visual_slam_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        node,
    ])
