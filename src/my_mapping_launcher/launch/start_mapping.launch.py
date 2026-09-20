import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_mapping_launcher')
    # 获取 yaml 配置文件路径
    config_dir = os.path.join(
        get_package_share_directory('my_mapping_launcher'),
        'config',
        'navsat_params.yaml'
    )
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'mapping.rviz')

    return LaunchDescription([
        
        # ---------------------------------------------------------
        # 1. 核心搭桥 TF：修复 Point-LIO 的坐标系断裂问题
        # 将 TF 树中的 'aft_mapped' 完美桥接到里程计声明的 'body' (全填0即可)
        # ---------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_aft_mapped_to_body',
            arguments=[
                '--x', '0.0', 
                '--y', '0.0', 
                '--z', '0.0', 
                '--yaw', '0.0', 
                '--pitch', '0.0', 
                '--roll', '0.0', 
                '--frame-id', 'aft_mapped', 
                '--child-frame-id', 'body'
            ]
        ),

        # ---------------------------------------------------------
        # 2. 传感器偏移 TF：解决“杆臂效应”
        # 发布雷达中心 (body) 到 GPS天线 (gps_link) 的固定物理距离
        # 这里的参数需要你拿卷尺实测！假设 GPS 在雷达后方 0.2m，上方 0.5m
        # ---------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_body_to_gps',
            arguments=[
                '--x', '-0.2', 
                '--y', '0.0', 
                '--z', '0.5', 
                '--yaw', '0.0', 
                '--pitch', '0.0', 
                '--roll', '0.0', 
                '--frame-id', 'body', 
                '--child-frame-id', 'gps_link'
            ]
        ),

        # ---------------------------------------------------------
        # 3. 启动 UTM 全局融合节点
        # ---------------------------------------------------------
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[config_dir],
            remappings=[
                # 将节点默认订阅的话题，映射为你实际传感器和算法输出的话题
                ('imu/data', '/unilidar/imu'),          # 你的 6 轴 IMU 话题
                ('gps/fix', '/gps/fix'),                # 你的 GPS 话题
                ('odometry/filtered', '/aft_mapped_to_init')      # Point-LIO 输出的局部里程计话题
            ]
        ),
        # 自动启动 RViz2 并加载专属配置！
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])
