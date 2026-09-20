from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')
    sensor_z_min_arg = DeclareLaunchArgument(
        'sensor_z_min', default_value='-1.5',
        description='Minimum finite LiDAR z value in the sensor frame.')
    point_filter_num_arg = DeclareLaunchArgument(
        'point_filter_num', default_value='2',
        description='Point-LIO input decimation factor.')
    lidar_meas_cov_arg = DeclareLaunchArgument(
        'lidar_meas_cov', default_value='0.01',
        description='Point-LIO LiDAR measurement covariance.')
    imu_acc_cov_arg = DeclareLaunchArgument(
        'imu_meas_acc_cov', default_value='0.1',
        description='Point-LIO IMU acceleration measurement covariance.')
    imu_gyro_cov_arg = DeclareLaunchArgument(
        'imu_meas_omg_cov', default_value='0.1',
        description='Point-LIO IMU angular-rate measurement covariance.')

    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'vrx_wamv.yaml'
        ]),
        {
            'use_sim_time': True,  # VRX publishes /clock
            # Use the 100 Hz VRX IMU for state propagation. Pure point-wise
            # LiDAR updates can momentarily jump in geometrically degenerate
            # water scenes even after the artificial water plane is hidden.
            'use_imu_as_input': True,
            'prop_at_freq_of_imu': True,
            # IMU stationary detector + zero-velocity update. Position hold is
            # intentionally disabled because IMU cannot measure absolute pose.
            'mapping.zupt_enable': True,
            'mapping.zupt_position_hold_enable': False,
            'mapping.zupt_gyro_threshold': 0.03,
            'mapping.zupt_acc_threshold': 0.20,
            'mapping.zupt_gyro_std_threshold': 0.01,
            'mapping.zupt_acc_std_threshold': 0.10,
            'mapping.zupt_window_size': 60,
            'mapping.zupt_min_stationary_time': 0.50,
            'mapping.zupt_velocity_cov': 0.01,
            'mapping.zupt_position_cov': 0.0025,
            'check_satu': True,
            'init_map_size': 10,
            # 32-beam cloud is 2.3x the 16-beam one; point_filter_num 1 + 0.2 voxels
            # overruns one core (processing lag grows unbounded -> ESKF drifts).
            # 2 / 0.3 keeps lag flat while still ~2x denser than the original 4 / 0.5.
            # Stationary VRX A/B testing selected filter=2, LiDAR covariance
            # 0.01, and sensor_z_min=-1.5 as the buoy-preserving default combination.
            'point_filter_num': ParameterValue(
                LaunchConfiguration('point_filter_num'), value_type=int),
            'space_down_sample': True,
            'filter_size_surf': 0.3,
            'filter_size_map': 0.3,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False,
            'mapping.lidar_meas_cov': ParameterValue(
                LaunchConfiguration('lidar_meas_cov'), value_type=float),
            'mapping.imu_meas_acc_cov': ParameterValue(
                LaunchConfiguration('imu_meas_acc_cov'), value_type=float),
            'mapping.imu_meas_omg_cov': ParameterValue(
                LaunchConfiguration('imu_meas_omg_cov'), value_type=float),
        }
    ]

    cloud_filter_node = Node(
        package='lidar_timestamp_adapter',
        executable='cloud_filter',
        name='cloud_filter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # The click plane is hidden at the Gazebo source. Also reject the
            # Remaining low-angle wave/surface returns still destabilize the
            # stationary scan matcher.  The sensor is ~1.8 m above water;
            # keep the cutoff at -1.5 m: direct VRX samples place marker-buoy
            # returns around -1.33 m and round-buoy returns near -1.1 m, while
            # the water sheet is around -1.8 m. Raising it to -1.2 m deletes
            # the marker-buoy returns from the Point-LIO input.
            'reject_below_sensor_z': True,
            'sensor_z_min': ParameterValue(
                LaunchConfiguration('sensor_z_min'), value_type=float),
        }],
    )

    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
    )

    # Point-LIO publishes camera_init -> aft_mapped.  Own the static
    # body-frame bridge in the localization launch (not Nav2), so mapping and
    # obstacle filtering have one connected TF tree before navigation starts.
    aft_mapped_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aft_mapped_to_base_link',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'aft_mapped',
            '--child-frame-id', 'wamv/wamv/base_link',
        ],
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='lio_rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    return LaunchDescription([
        rviz_arg,
        sensor_z_min_arg,
        point_filter_num_arg,
        lidar_meas_cov_arg,
        imu_acc_cov_arg,
        imu_gyro_cov_arg,
        cloud_filter_node,
        laser_mapping_node,
        aft_mapped_to_base_link,
        rviz_node,
    ])
