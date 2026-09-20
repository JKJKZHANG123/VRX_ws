from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
import os

def generate_launch_description():
    # 获取当前包路径
    package_path = os.path.dirname(os.path.realpath(__file__))
    
    return LaunchDescription([
        # GPS驱动节点
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            output='screen',
            name='gps_driver',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 9600,
                'frame_id': 'gps_link',
            }],
            arguments=['--ros-args', '--log-level', 'error'],
            remappings=[
                ('/fix', '/gps/fix'),
                ('/vel', '/gps/vel'),
            ]
        ),
        
        # GPS轨迹节点
        Node(
            package='my_gps_driver',
            executable='gps_trajectory_node',
            output='screen',
            name='gps_trajectory',
            parameters=[{
                'window_size': 5,
                'max_path_length': 1000,
                'scale_factor': 100000.0,
            }]
        ),
        
        # 静态坐标变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'gps_link'],
            output='screen'
        ),
        
        # 启动RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', [FindExecutable(name='rviz2'), '--display-config', '']],
            parameters=[{
                'use_sim_time': False,
            }]
        ),
    ])