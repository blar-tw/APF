import os
from glob import glob

from setuptools import setup

package_name = 'apf_oa'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blar',
    maintainer_email='terryph1205@gmail.com',
    description='Artificial Potential Field obstacle avoidance for a PX4 multirotor in Gazebo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_sensor_node = apf_oa.obstacle_sensor_node:main',
            'apf_planner_node = apf_oa.apf_planner_node:main',
        ],
    },
)
