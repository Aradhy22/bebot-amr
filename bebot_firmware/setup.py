import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bebot_firmware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bebot',
    maintainer_email='aradhyagarwal242@gmail.com',
    description='Serial bridge and encoder odometry node for the bebot AMR (Jetson <-> Arduino Uno)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_odometry_node = bebot_firmware.serial_odometry_node:main',
            'teleop_custom = bebot_firmware.teleop_custom:main',
        ],
    },
)
