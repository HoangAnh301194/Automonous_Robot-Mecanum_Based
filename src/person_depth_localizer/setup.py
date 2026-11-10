from glob import glob
from setuptools import find_packages, setup

package_name = 'person_depth_localizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics project',
    maintainer_email='hoanganh94ptit2k5@gmail.com',
    description='Uncalibrated person localization preview; not for autonomous navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rough_localizer = person_depth_localizer.rough_localizer:main',
        ],
    },
)
