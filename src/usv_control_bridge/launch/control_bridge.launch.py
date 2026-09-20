from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('usv_control_bridge'), 'config', 'bridge_params.yaml'])

    return LaunchDescription([
        Node(
            package='usv_control_bridge',
            executable='control_bridge',
            name='control_bridge',
            output='screen',
            parameters=[params],
        )
    ])
