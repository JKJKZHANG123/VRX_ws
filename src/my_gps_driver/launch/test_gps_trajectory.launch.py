from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 只启动轨迹节点（用于测试，假设GPS数据已经发布）
        Node(
            package='my_gps_driver',
            executable='gps_trajectory_node',
            output='screen',
            name='gps_trajectory',
            parameters=[{
                'window_size': 3,
                'max_path_length': 500,
                'scale_factor': 100000.0,
            }]
        ),
    ])