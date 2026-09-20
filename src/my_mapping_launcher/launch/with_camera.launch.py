"""
完整建图 + 彩色点云 + 视觉回环 (无 GPS)

启动顺序:
  t=0s:  LiDAR 驱动
  t=1s:  相机驱动 (Orbbec)
  t=3s:  Point-LIO
  t=5s:  点云着色 + 视觉回环检测
  t=7s:  RViz2
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    point_lio_dir = get_package_share_directory('point_lio')
    mapping_launcher_dir = get_package_share_directory('my_mapping_launcher')
    lcf_dir = get_package_share_directory('lidar_camera_fusion')
    vlc_dir = get_package_share_directory('visual_loop_closure')

    # 可选参数
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='启动 RViz2')
    colorize_arg = DeclareLaunchArgument(
        'colorize', default_value='true',
        description='启动点云着色')
    loop_closure_arg = DeclareLaunchArgument(
        'loop_closure', default_value='true',
        description='启动视觉回环检测')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='demo_rgb_color.rviz',
        description='RViz 配置文件名 (rviz/ 目录下)')

    # ===== 传感器驱动 =====

    # LiDAR 驱动
    lidar_driver = Node(
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

    # Orbbec 相机驱动
    camera_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('orbbec_camera'),
                         'launch', 'astra_pro_plus.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'false',  # 我们自己做着色
            'color_width': '640',
            'color_height': '480',
            'depth_width': '640',
            'depth_height': '480',
            'publish_tf': 'true',
        }.items(),
    )

    # ===== 建图算法 =====

    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_dir, 'launch', 'mapping_unilidar_l1.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items(),  # 我们用外层的 rviz
    )

    # ===== 视觉增强 =====

    # 点云着色
    colorizer_node = Node(
        package='lidar_camera_fusion',
        executable='pointcloud_colorizer',
        name='pointcloud_colorizer',
        output='screen',
        parameters=[os.path.join(lcf_dir, 'config', 'colorizer_params.yaml')],
        condition=IfCondition(LaunchConfiguration('colorize')),
    )

    # 视觉回环检测
    loop_closure_node = Node(
        package='visual_loop_closure',
        executable='visual_slam_node',
        name='visual_slam_node',
        output='screen',
        parameters=[os.path.join(vlc_dir, 'config', 'loop_closure_params.yaml')],
        condition=IfCondition(LaunchConfiguration('loop_closure')),
    )

    # ===== RViz2 =====
    # 展示配置可切换: demo_rgb_color.rviz (仅彩色) / demo_combined.rviz (白色+彩色)
    rviz_config = PathJoinSubstitution([
        mapping_launcher_dir, 'rviz', LaunchConfiguration('rviz_config')])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ===== 编排启动顺序 =====
    return LaunchDescription([
        rviz_arg,
        colorize_arg,
        loop_closure_arg,
        rviz_config_arg,

        # t=0s: 传感器
        lidar_driver,
        TimerAction(period=1.0, actions=[camera_driver]),

        # t=3s: 建图
        TimerAction(period=3.0, actions=[point_lio_launch]),

        # t=5s: 视觉增强
        TimerAction(period=5.0, actions=[colorizer_node]),
        TimerAction(period=5.0, actions=[loop_closure_node]),

        # t=7s: 可视化
        TimerAction(period=7.0, actions=[rviz_node]),
    ])
