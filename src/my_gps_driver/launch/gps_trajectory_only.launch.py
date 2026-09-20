from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'gps_topic',
            default_value='/gps/fix',
            description='GPS数据话题'
        ),
        
        DeclareLaunchArgument(
            'path_topic',
            default_value='/gps/path',
            description='轨迹路径话题'
        ),
        
        DeclareLaunchArgument(
            'window_size',
            default_value='5',
            description='轨迹平滑窗口大小'
        ),
        
        DeclareLaunchArgument(
            'max_path_length',
            default_value='1000',
            description='最大轨迹点数量'
        ),
        
        DeclareLaunchArgument(
            'scale_factor',
            default_value='100000.0',
            description='经纬度缩放因子'
        ),
        
        # GPS轨迹节点
        Node(
            package='my_gps_driver',
            executable='gps_trajectory_node',
            output='screen',
            name='gps_trajectory',
            parameters=[{
                'window_size': LaunchConfiguration('window_size'),
                'max_path_length': LaunchConfiguration('max_path_length'),
                'scale_factor': LaunchConfiguration('scale_factor'),
            }],
            remappings=[
                ('/gps/fix', LaunchConfiguration('gps_topic')),
                ('/gps/path', LaunchConfiguration('path_topic')),
            ]
        ),
    ])