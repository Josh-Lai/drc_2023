from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    rgb_mask_node = Node(
        package="vision",
        executable="mask_rgb",
    )

    depth_mask_node = Node(
        package="vision",
        executable="mask_depth"
    )

    ld.add_action(rgb_mask_node)
    ld.add_action(depth_mask_node)

    return ld