import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    cv_detection_config_file = os.path.join(
        get_package_share_directory("vision"), "config", "params.yaml"
    )
    cv_detection = Node(
        package="vision",
        executable="laneDetection",
        parameters=[cv_detection_config_file],
    )
    # publisher_node = Node(
    #     package="vision",
    #     executable="img_publisher"
    # )

    ld.add_action(cv_detection)
    # ld.add_action(publisher_node)
    return ld
