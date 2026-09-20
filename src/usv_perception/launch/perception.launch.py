from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('usv_perception'), 'config', 'perception_params.yaml'])

    # 三个方向各一个开关, 默认全开
    buoy_arg = DeclareLaunchArgument('buoy', default_value='true',
                                     description='方向1: 浮标语义识别')
    color_arg = DeclareLaunchArgument('colorize', default_value='true',
                                      description='方向2: 彩色点云')
    water_arg = DeclareLaunchArgument('water', default_value='true',
                                      description='方向3: 水面点云过滤')

    common = [params, {'use_sim_time': True}]

    buoy_node = Node(
        package='usv_perception', executable='buoy_detector',
        name='buoy_detector', output='screen', parameters=common,
        condition=IfCondition(LaunchConfiguration('buoy')))

    color_node = Node(
        package='usv_perception', executable='cloud_colorizer',
        name='cloud_colorizer', output='screen', parameters=common,
        condition=IfCondition(LaunchConfiguration('colorize')))

    water_node = Node(
        package='usv_perception', executable='water_filter',
        name='water_filter', output='screen', parameters=common,
        condition=IfCondition(LaunchConfiguration('water')))

    return LaunchDescription([
        buoy_arg, color_arg, water_arg,
        buoy_node, color_node, water_node,
    ])
