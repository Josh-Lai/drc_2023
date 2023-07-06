import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # controller_config_file = os.path.join(
    #     get_package_share_directory("vision"), "config", "parameters.yaml"
    # )

    cv_detection = Node(
        package="cv_detection",
        executable="opencv_laneDetection",
        parameters=[{"image_topic": "camera/color/image_raw"}]
    )
    # controller_node = Node(
    #     package="controller",
    #     executable="controller",
    #     parameters=[controller_config_file]
    # )
    ld.add_action(cv_detection)
    # ld.add_action(controller_node)
    return ld
