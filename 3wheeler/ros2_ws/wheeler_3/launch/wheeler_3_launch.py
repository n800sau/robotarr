import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
#	param_file_name = 'wheeler_3.yaml'
#	param_dir = LaunchConfiguration(
#		'params',
#		default=os.path.join(
#			get_package_share_directory('wheeler_3'),
#			'param',
#			param_file_name))
#	launch_file_dir = os.path.join(get_package_share_directory('wheeler_3'))
	return LaunchDescription([
#		DeclareLaunchArgument(
#			'params',
#			default_value=param_dir,
#			description='Full path to param file to load'),

#		IncludeLaunchDescription(
#			PythonLaunchDescriptionSource([launch_file_dir, '/wheeler_3_launch.py']),
#			launch_arguments={
#				'params': param_dir}.items(),
#		),

# map with odom is for slam
#		Node(
#			package='tf2_ros',
#			executable='static_transform_publisher',
#			output='screen',
#			name='odom_map_drift',
#			arguments=["--x", "0", "--y", "0", "--z", "0", '--yaw', '0', '--pitch', '0', '--roll', '0',  "--frame-id", "map", "--child-frame-id", "odom"]
#		),
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			output='screen',
			name='world_map',
			arguments=["--x", "0", "--y", "0", "--z", "0", '--yaw', '0', '--pitch', '0', '--roll', '0', "--frame-id", "map", "--child-frame-id", "world"]
		),
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			name='base_laser',
			arguments = ['--x', '0', '--y', '0', '--z', '0.1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base-link', '--child-frame-id', 'laser']
		),
		Node(
			package='wheeler_3',
			executable='wheeler_3',
			name='wheeler_3',
			parameters=[{
#				'serial_port': '/dev/serial/by-id/usb-STMicroelectronics_BLUEPILL_F103C8_CDC_in_FS_Mode_8D8351A25350-if00',
				'serial_port': '/dev/serial/by-id/usb-STMicroelectronics_BLUEPILL_F103C8_CDC_in_FS_Mode_6D87477B5656-if00',
			}]
		),
#		Node(
#			package='wheeler_3',
#			executable='wheeler_streamer_3',
#			name='wheeler_streamer_3'
#		),
		Node(
			name='rplidar_composition',
			package='rplidar_ros',
			executable='rplidar_composition',
			output='screen',
			parameters=[{
				'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
				'serial_baudrate': 115200,  # A1 / A2
				# 'serial_baudrate': 256000, # A3
				'frame_id': 'laser',
				'inverted': False,
				'angle_compensate': True,
			}],
		),
		ComposableNodeContainer(
			name='gscam_container',
			namespace='',
			package='rclcpp_components',
			executable='component_container',
			composable_node_descriptions=[
				# GSCam driver
				ComposableNode(
					package='gscam',
					plugin='gscam::GSCam',
					name='gscam_node',
					parameters=[{
						'gscam_config': 'curlhttpsrc location=http://wintel.local:8080/stream ! multipartdemux ! image/jpeg,width=640,height=480 ! jpegdec ! videosink',
#						'camera_info_url': 'package://gscam/examples/uncalibrated_parameters.ini',
					}],
					# Future-proof: enable zero-copy IPC when it is available
					# https://github.com/ros-perception/image_common/issues/212
					extra_arguments=[{'use_intra_process_comms': True}],
				),
			],
		)

#		IncludeLaunchDescription(
#			PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rplidar_ros')), '/launch/rplidar.launch.py']),
#			launch_arguments={
#				'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
#			}.items()
#		),
	])
