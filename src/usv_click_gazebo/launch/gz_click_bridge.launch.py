"""Bridge Gazebo marker/waypoint topics to ROS and run the conversion node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_click_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # marker point (world frame) from the plugin
            '/gazebo/click/point@geometry_msgs/msg/Vector3[gz.msgs.Vector3d',
            # boat world pose (from plugin)
            '/gazebo/boat_pose@geometry_msgs/msg/Vector3[gz.msgs.Vector3d',
            # waypoint control commands from QML buttons
            '/gazebo/waypoint_cmd@std_msgs/msg/String[gz.msgs.StringMsg',
        ],
        remappings=[
            ('/gazebo/click/point', '/gz_click/point'),
            ('/gazebo/boat_pose', '/gz_boat_pose'),
            ('/gazebo/waypoint_cmd', '/gz_waypoint_cmd'),
        ],
    )

    node = Node(
        package='usv_click_gazebo',
        executable='gz_click_to_goal_node.py',
        name='gz_click_to_goal_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([bridge, node])
