from setuptools import find_packages, setup

package_name = 'robot_task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_task_manager.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour',
    maintainer_email='nour.el.bachari@accenture.com',
    description='Orchestrates pick/place tasks for the robot pipeline',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'pick_task_node = robot_task_manager.pick_task_node:main',
            'camera_buffer_node = robot_task_manager.camera_buffer_node:main',
            'visualize_segmentation_node = robot_task_manager.visualize_segmentation_node:main',
            'pointcloud_node = robot_task_manager.pointcloud_node:main',
            'visualize_grasps_node = robot_task_manager.visualize_grasps_node:main',
            'motion_node = robot_task_manager.motion_node:main',
            'scene_publisher_node = robot_task_manager.scene_publisher_node:main',
        ],
    },
)
