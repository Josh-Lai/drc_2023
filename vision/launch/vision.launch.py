import os 
import yaml 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def yaml_to_dict(path_to_yaml):
	with open(path_to_yaml, "r") as f:
		return yaml.load(f, Loader=yaml.SafeLoader)

package_path = get_package_share_directory('vision')
camera_parameters = yaml_to_dict(f"{package_path}/param/camera.yaml") 
print(camera_parameters)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
			output='screen',
			parameters = [camera_parameters]
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
			parameters = [
				{"send_buffer_limit": 10000000000}
			] 
        ),
    ])
