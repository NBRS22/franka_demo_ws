import glob

from setuptools import find_packages, setup

package_name = 'handeye_tf_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/publish.launch.py']),
        # AprilTag config files (family/size/detector settings) -- installed
        # so other packages (e.g. fp3_apriltag_demo) can reference them via
        # FindPackageShare('handeye_tf_publisher') instead of a hardcoded
        # source-tree path.
        ('share/' + package_name + '/tags', glob.glob('tags/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour el bachari',
    maintainer_email='n.elbachari@gmail.com',
    description=(
        'Publishes fp3_link0 -> camera_link as a static transform from '
        'an easy_handeye2 calibration and the realsense2_camera internal TF.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handeye_tf_publisher = handeye_tf_publisher.publisher_node:main',
            'calibration_pose_tour = handeye_tf_publisher.calibration_pose_tour_node:main',
        ],
    },
)
