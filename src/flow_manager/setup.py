from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'flow_manager'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer_email='ngr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'flow_manager_node = flow_manager.flow_manager_node:main',
            'camera_buffer_node = flow_manager.camera_buffer_node:main',
            'pointcloud_node = flow_manager.pointcloud_node:main',
            'grasp_selector_node = flow_manager.grasp_selector_node:main',
            'task_validator_node = flow_manager.task_validator_node:main',
            'check_prerequisites = flow_manager.check_prerequisites:main',
        ],
    },
)
