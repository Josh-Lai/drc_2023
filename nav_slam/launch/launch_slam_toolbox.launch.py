import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory("nav_slam"), "config", "slam_params.yaml")
    depth_to_scan_config = os.path.join(
        get_package_share_directory('nav_slam'), 'config', 'depth_conversion_params.yaml')
    laser_scan_config = os.path.join(
        get_package_share_directory('nav_slam'), 'config', 'laser_match_param.yaml')
    point_cloud_to_laserscan_config = os.path.join(
        get_package_share_directory("nav_slam"), "config" ,"lane_points_to_scan.yaml")

    start_slam_node = Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node", 
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params]
    )

    start_lane_points_to_scan = Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            remappings=[
                ("cloud_in", "lane_points"),
                ("scan", "lane_scan")],
            parameters=[point_cloud_to_laserscan_config],
            name="pointcloud_to_laserscan")
    
    depth_to_scan = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth', '/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/depth/camera_info')],
            parameters=[depth_to_scan_config],
            output="screen")

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

    #ld.add_action(start_cam_depth_cvt)
    ld.add_action(depth_to_scan)
    ld.add_action(start_slam_node)
    ld.add_action(start_scan_matcher)
    ld.add_action(cam_link);
    ld.add_action(base_footprint);
    ld.add_action(start_lane_points_to_scan)
    return ld

