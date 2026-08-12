import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fp3_apriltag_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngr',
    maintainer_email='n.elbachari@gmail.com',
    description=(
        'v1: reads one AprilTag detection from /detections, estimates its 3D pose via '
        'solvePnP, transforms it into fp3_link0, and sends a single MoveToPose goal to '
        'fp3_moveit_server'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'apriltag_move_once_node = fp3_apriltag_demo.apriltag_move_once_node:main',
        ],
    },
)
