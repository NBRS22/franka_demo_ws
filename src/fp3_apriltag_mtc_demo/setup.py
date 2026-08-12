import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fp3_apriltag_mtc_demo'

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
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngr',
    maintainer_email='n.elbachari@gmail.com',
    description=(
        'AprilTag -> 3D pose -> fp3_link0 -> two grasp candidates handed to '
        'fp3_moveit_server\'s MTC-based pick_object action, which owns the entire '
        'pick+place+gripper lifecycle itself'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'apriltag_pick_once_node = fp3_apriltag_mtc_demo.apriltag_pick_once_node:main',
        ],
    },
)
