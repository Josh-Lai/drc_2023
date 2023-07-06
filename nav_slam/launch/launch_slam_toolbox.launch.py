import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    slam_params = os.path.join(get_package_share_directory("nav_slam"), "config", "slam_params.yaml")
    start_slam_node = Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node", 
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params]
    )
    ld = LaunchDescription()

    ld.add_action(start_slam_node)
    return ld

