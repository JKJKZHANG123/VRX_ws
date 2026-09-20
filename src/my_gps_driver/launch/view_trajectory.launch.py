from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    return LaunchDescription([
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