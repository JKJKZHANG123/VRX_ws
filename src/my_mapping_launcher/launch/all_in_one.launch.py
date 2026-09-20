import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ==========================================
    # 1. 获取各个核心包的路径
    # ==========================================
    point_lio_dir = get_package_share_directory('point_lio')
    mapping_launcher_dir = get_package_share_directory('my_mapping_launcher')

    # ==========================================
    # 2. 定义各个节点的启动动作
    # ==========================================
    
    # 动作 A: 启动底层雷达与 IMU 驱动节点
    lidar_driver_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen'
    )

    # 动作 B: 启动官方 GPS 驱动节点 (已完美移植你的原版配置！)
    gps_driver_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        name='gps_driver',
        parameters=[{
            'port': '/dev/ttyUSB1',  # 注意：这里按你原文件写的是 USB0
            'baud': 9600,
            'frame_id': 'gps_link',
        }],
        # 屏蔽校验和警告
        arguments=['--ros-args', '--log-level', 'error'],
        remappings=[
            ('/fix', '/gps/fix'),
            ('/vel', '/gps/vel'),
        ]
    )

    # 动作 C: 启动 Point-LIO 核心算法 (复用它自带的 launch)
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_dir, 'launch', 'mapping_unilidar_l1.launch.py')
        )
    )

    # 动作 D: 启动我们之前写好的 UTM 融合与 RViz 可视化
    utm_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_launcher_dir, 'launch', 'start_mapping.launch.py')
        )
    )

    # ==========================================
    # 3. 编排启动顺序
    # ==========================================
    return LaunchDescription([
        # 第 0 秒：立刻启动底层的感知器官 (雷达、IMU、GPS)
        lidar_driver_node,
        gps_driver_node,
        
        # 第 2 秒：等传感器数据稳定后，启动建图大脑 Point-LIO
        TimerAction(
            period=2.0,
            actions=[point_lio_launch]
        ),

        # 第 4 秒：等局部地图产生后，启动 UTM 坐标系融合与 RViz 界面
        TimerAction(
            period=4.0,
            actions=[utm_fusion_launch]
        )
    ])
