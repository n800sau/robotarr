import os
from glob import glob
from setuptools import setup

package_name = 'wheeler_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('param/*.yaml')),
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
            'wheeler_3 = wheeler_3.wheeler_3:main',
            'wheeler_streamer_3 = wheeler_3.streamer3:main',
            'teleop_keyboard = wheeler_3.teleop_keyboard:main',
        ],
    },
)
