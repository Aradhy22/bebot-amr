import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bebot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bebot',
    maintainer_email='aradhyagarwal242@gmail.com',
    description='Camera-based object detection, tracking, and distance estimation for the bebot AMR (YOLOv8n)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_node = bebot_vision.object_detection_node:main',
        ],
    },
)
