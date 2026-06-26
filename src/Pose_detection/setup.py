from setuptools import setup
import os

package_name = 'pose_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'gesture'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='orin@todo.todo',
    description='Pose detection and gesture recognition',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_ros_node = pose_detection.pose_ros_node:main'
        ],
    },
)
