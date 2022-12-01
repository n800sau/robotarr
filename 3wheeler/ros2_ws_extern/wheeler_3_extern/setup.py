import os
from glob import glob
from setuptools import setup

package_name = 'wheeler_3_extern'

setup(
	name=package_name,
	version='0.0.0',
	packages=[package_name],
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
		('share/' + package_name + '/worlds', [
			'worlds/wheeler_3_world.wbt', 'worlds/.wheeler_3_world.wbproj',
		]),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='n800s',
	maintainer_email='n800sau@gmail.com',
	description='TODO: Package description',
	license='TODO: License declaration',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
		],
	},
)
