import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'underwater_motor_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhiram P T',
    maintainer_email='ptabhiram449@gmail.com',
    description='Motor firmware simulator and ROS 2 bridge: cmd_vel to RPM telemetry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_sim_node = underwater_motor_sim.motor_sim_node:main',
        ],
    },
)
