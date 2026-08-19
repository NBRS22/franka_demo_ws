from setuptools import find_packages, setup

package_name = 'fp3_apriltag_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/apriltag_move_once.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour el bachari',
    maintainer_email='n.elbachari@gmail.com',
    description=(
        'Real-hardware eye-on-base calibration check: moves the arm to an '
        'AprilTag detected by the camera and grasps it via mtc_pick.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_move_once_node = fp3_apriltag_demo.apriltag_move_once_node:main',
        ],
    },
)
