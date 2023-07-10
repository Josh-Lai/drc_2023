import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory("nav_slam"), "config", "slam_params.yaml")
    pointcloud_to_laser_config = os.path.join(
            get_package_share_directory("nav_slam"), "config", "lane_pointcloud_to_scan.yaml")
    depth_to_scan_config = os.path.join(
        get_package_share_directory('nav_slam'), 'config', 'depth_conversion_params.yaml')
    laser_scan_config = os.path.join(
        get_package_share_directory('nav_slam'), 'config', 'laser_match_param.yaml')

    start_slam_node = Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node", 
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params]
    )
    start_lane_depth_cvt = Node(
            package="nav_slam",
            executable="cam_to_scan",
            name="cam_to_scan"
    )
    
    depth_to_scan = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth', '/camera/depth/image_rect_raw'),
                        ('scan', 'depthscan'),
                        ('depth_camera_info', '/camera/depth/camera_info')],
            parameters=[depth_to_scan_config])

    lane_pointcloud_to_scan = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/lane_points_flat'),
                        ('scan', '/lanescan')],
            parameters=[pointcloud_to_laser_config],
            name='pointcloud_to_laserscan'
        )

    start_scan_matcher = Node(
            package="ros2_laser_scan_matcher",
            executable="laser_scan_matcher",
            name="laser_scan_matcher",
            parameters=[laser_scan_config],
            output="screen")


    cam_link = Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
    )
    base_footprint = Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'camera_link', '--child-frame-id', 'base_footprint']
    )

    ld = LaunchDescription()

    ld.add_action(start_lane_depth_cvt)
    ld.add_action(depth_to_scan)
    ld.add_action(start_slam_node)
    ld.add_action(start_scan_matcher)
    ld.add_action(cam_link)
    ld.add_action(base_footprint)
    ld.add_action(lane_pointcloud_to_scan)
    return ld

