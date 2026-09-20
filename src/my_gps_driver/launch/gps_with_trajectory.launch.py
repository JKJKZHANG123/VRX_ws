from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='GPS串口设备路径'
        ),
        
        DeclareLaunchArgument(
            'baud',
            default_value='9600',
            description='串口波特率'
        ),
        
        DeclareLaunchArgument(
            'window_size',
            default_value='10',  # 增大窗口大小
            description='轨迹平滑窗口大小'
        ),
        
        DeclareLaunchArgument(
            'max_path_length',
            default_value='1000',
            description='最大轨迹点数量'
        ),
        
        DeclareLaunchArgument(
            'scale_factor',
            default_value='50000.0',  # 减小缩放因子
            description='经纬度缩放因子'
        ),
        
        DeclareLaunchArgument(
            'min_distance_threshold',
            default_value='1.0',  # 最小移动距离阈值（米）
            description='忽略微小移动的阈值'
        ),
        
        DeclareLaunchArgument(
            'min_accuracy_threshold',
            default_value='5.0',  # 精度阈值（米）
            description='GPS精度阈值，高于此值的数据将被忽略'
        ),
        
        # GPS驱动节点
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            output='screen',
            name='gps_driver',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
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
                'window_size': LaunchConfiguration('window_size'),
                'max_path_length': LaunchConfiguration('max_path_length'),
                'scale_factor': LaunchConfiguration('scale_factor'),
                'min_distance_threshold': LaunchConfiguration('min_distance_threshold'),
                'min_accuracy_threshold': LaunchConfiguration('min_accuracy_threshold'),
            }]
        ),
        
        # 静态坐标变换（将gps_link连接到map坐标系）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'gps_link'],
            output='screen'
        ),
    ])