import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'hand_wave_detection'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'config.yaml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='HoangAnh301194@users.noreply.github.com',
    description='ROS 2 raised-hand detection using YOLO11 and RTMPose.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'hand_wave_detector = hand_wave_detection.ros_node:main',
        ],
    },
)
