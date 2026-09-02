from setuptools import find_packages, setup

package_name = 'depth_obstacle_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='HoangAnh301194@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'debugger_gui = depth_obstacle_detector.debugger_gui:main',
            'obstacle_detector = depth_obstacle_detector.obstacle_detector:main',
        ],
    },
)
