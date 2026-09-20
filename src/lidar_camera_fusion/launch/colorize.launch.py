"""
启动点云着色节点
可独立使用, 也可以被其他 launch 文件 include
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_camera_fusion')
    config = os.path.join(pkg_dir, 'config', 'colorizer_params.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='启动 RViz2 查看彩色点云')

    colorizer_node = Node(
        package='lidar_camera_fusion',
        executable='pointcloud_colorizer',
        name='pointcloud_colorizer',
        output='screen',
        parameters=[config],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_color',
        arguments=['-d', os.path.join(pkg_dir, '..', '..',
                     'point_lio_ros2', 'rviz_cfg', 'loam_livox.rviz')],
        condition=None,  # 默认不启动, 用户通过 rviz:=true 启用
    )

    return LaunchDescription([
        rviz_arg,
        colorizer_node,
    ])
