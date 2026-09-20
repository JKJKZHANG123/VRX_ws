from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'rotate_yaw_bias': 0.0,
            'range_scale': 0.001,
            'range_bias': 0.0,
            'range_max': 50.0,
            'range_min': 0.0,
            'cloud_frame': 'unilidar_lidar',
            'cloud_topic': 'unilidar/cloud',
            'cloud_scan_num': 54,
            'imu_frame': 'unilidar_imu',
            'imu_topic': 'unilidar/imu',
        }]
    )
    return LaunchDescription([node])
