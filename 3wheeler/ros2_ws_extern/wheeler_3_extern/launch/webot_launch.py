import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
	package_dir = get_package_share_directory('wheeler_3_extern')
	world = LaunchConfiguration('world')
	webots = WebotsLauncher(
		world=PathJoinSubstitution([package_dir, 'worlds', world])
	)

	return LaunchDescription([
		DeclareLaunchArgument(
			'world',
			default_value='wheeler_3_world.wbt',
			description='Choose one of the world files from `/wheeler_3_extern/worlds` directory'
		),
		webots,
	])
