import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'layer1_bringup'

setup(
    name=package_name,
    version='0.0.1',
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
    maintainer='orin',
    maintainer_email='orin@todo.todo',
    description='Unified Layer 1 bringup package with ESP32 bridge and 3-source EKF odometry fusion.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp_bridge = layer1_bringup.esp_bridge_node:main',
        ],
    },
)
