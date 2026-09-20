import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare('usv_navigation')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    )
    enable_dynamic_tracker = LaunchConfiguration(
        'enable_dynamic_tracker', default='false')

    # Lifecycle manager for all Nav2 nodes
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'smoother_server',
    ]

    # Create the launch description
    ld = LaunchDescription()

    # Resolve separate single- and multi-goal BT XML files. Both avoid unsafe
    # spin/back-up recovery, but they use different Nav2 blackboard contracts.
    # IMPORTANT: param_rewrites keys must be fully qualified (node.param_name).
    pkg_share_abs = get_package_share_directory('usv_navigation')
    bt_to_pose_abs = os.path.join(pkg_share_abs, 'config', 'usv_nav_to_pose.xml')
    bt_through_poses_abs = os.path.join(
        pkg_share_abs, 'config', 'usv_nav_through_poses.xml')
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': bt_to_pose_abs,
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml':
                bt_through_poses_abs,
        },
        convert_types=True,
    )

    ld.add_action(DeclareLaunchArgument(
        'enable_dynamic_tracker',
        default_value='false',
        description='Start the dynamic-obstacle tracker for controlled A/B tests'))

    # Set use_sim_time parameter for all nodes
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))

    # Controller Server (RPP)
    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
    ))

    # Planner Server (Smac Hybrid-A*)
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    ))

    # Smoother Server
    ld.add_action(Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
    ))

    # Behavior Server
    ld.add_action(Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
    ))

    # BT Navigator
    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    ))

    # Waypoint Follower
    ld.add_action(Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
    ))

    # Velocity Smoother
    ld.add_action(Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
    ))

    # camera_init -> aft_mapped -> base_link is owned by Point-LIO's
    # VRX mapping launch. Navigation consumes that TF tree but does not publish
    # a duplicate static transform.

    # Costmap -> 3D obstacle markers (visualization: lift occupied cells to cubes)
    ld.add_action(Node(
        package='usv_navigation',
        executable='costmap_3d_markers',
        name='costmap_3d_markers',
        output='screen',
        parameters=[{'use_sim_time': True}],
    ))

    # Dynamic obstacle tracker: subtracts mapped shoreline/buoys from the
    # current raw scan, clusters new returns, estimates velocity with an
    # alpha-beta filter, and publishes an 8 s prediction cloud for the local
    # costmap.  It is a normal node (not a Nav2 lifecycle node) and publishes
    # an empty cloud when tracks expire.
    dynamic_tracker_params = PathJoinSubstitution([
        pkg_share, 'config', 'dynamic_obstacle_tracker.yaml'])
    ld.add_action(Node(
        package='usv_navigation',
        executable='dynamic_obstacle_tracker',
        name='dynamic_obstacle_tracker',
        output='screen',
        parameters=[dynamic_tracker_params],
        condition=IfCondition(enable_dynamic_tracker),
    ))

    # Goal gateway: validate footprint/costmap/geofence, then send the
    # NavigateToPose action.  It also consumes external /goal_pose messages,
    # so goto.sh and the Gazebo single-marker path use the same safety checks.
    goal_guard_params = PathJoinSubstitution([
        pkg_share, 'config', 'goal_guard.yaml'])
    ld.add_action(Node(
        package='usv_navigation',
        executable='click_to_goal',
        name='click_to_goal',
        output='screen',
        parameters=[goal_guard_params],
    ))

    # Do not launch feature_nav_node in the safety-critical command path.
    # It used to publish a second writer to /cmd_vel_smoothed and switch to
    # open-loop DIRECT mode when the cloud became sparse. That competed with
    # Nav2's velocity_smoother and could make the WAM-V turn in circles or
    # drive blindly into the shore. Nav2 is now the sole command authority;
    # feature_nav_node remains available only for offline experiments.

    # Lifecycle Manager
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ],
    ))

    return ld
