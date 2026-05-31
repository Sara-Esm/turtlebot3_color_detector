from setuptools import setup
from glob import glob
import os

package_name = 'turtlebot3_color_follower'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sara Esmaeili',
    maintainer_email='zesmaeili85@gmail.com',
    description='TurtleBot3 color follower using SMC visual servoing and two-node ROS 2 architecture.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector = turtlebot3_color_follower.color_detector_node:main',
            'smc_controller = turtlebot3_color_follower.smc_controller_node:main',
        ],
    },
)
