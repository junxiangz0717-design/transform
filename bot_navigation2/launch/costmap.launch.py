from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = "/home/tianze/rm_sim_ws/src/bot_navigation2/config/costmap.yaml"

    return LaunchDescription([
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            name="costmap",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_costmap",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["costmap/costmap"]
            }],
        ),
    ])