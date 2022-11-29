import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	param_file_name = 'wheeler_3.yaml'
	param_dir = LaunchConfiguration(
		'params',
		default=os.path.join(
			get_package_share_directory('wheeler_3'),
			'param',
			param_file_name))
	launch_file_dir = os.path.join(get_package_share_directory('wheeler_3'))
	return LaunchDescription([
		DeclareLaunchArgument(
			'params',
			default_value=param_dir,
			description='Full path to param file to load'),

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([launch_file_dir, '/wheeler_3_launch.py']),
			launch_arguments={
				'params': param_dir}.items(),
		),

		Node(
			package='wheeler_3',
			executable='wheeler_3',
			name='wheeler_3'
		)
	])
