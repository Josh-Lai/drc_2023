import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    cv_detection = Node(
        package="vision",
        executable="laneDetection",
        parameters=[{"image_topic": "camera/color/image_rawr"}]
        # parameters=[{"image_topic": "camera2/image_raw"}]
    )
    # publisher_node = Node(
    #     package="vision",
    #     executable="img_publisher"
    # )

    ld.add_action(cv_detection)
    # ld.add_action(publisher_node)
    return ld
