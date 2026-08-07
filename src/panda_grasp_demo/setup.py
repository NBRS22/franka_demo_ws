from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'panda_grasp_demo'

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
    description='v1: sends a hardcoded grasp pose to panda_motion_server (Isaac Sim)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hardcoded_grasp_node = panda_grasp_demo.hardcoded_grasp_node:main',
            'grasp_pose_subscriber_node = panda_grasp_demo.grasp_pose_subscriber_node:main',
        ],
    },
)
