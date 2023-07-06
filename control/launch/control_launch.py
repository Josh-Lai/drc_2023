import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    control_node = Node(
        package="control",
        executable="control",
        parameters=[{}]  # TODO: populate the parameters
    )

    ld.add_action(control_node)
    return ld
