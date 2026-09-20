from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            output='screen',
            name='gps_driver',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baud': 9600,
                'frame_id': 'gps_link',
            }],
            # 屏蔽校验和警告
            arguments=['--ros-args', '--log-level', 'error'],
            remappings=[
                ('/fix', '/gps/fix'),
                ('/vel', '/gps/vel'),
            ]
        ),
    ])
