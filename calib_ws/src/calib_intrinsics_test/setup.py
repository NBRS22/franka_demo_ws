import glob

from setuptools import find_packages, setup

package_name = 'calib_intrinsics_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/intrinsics_grid_check.launch.py']),
        ('share/' + package_name + '/tags', glob.glob('tags/*.yaml')),
        ('share/' + package_name + '/scripts', ['scripts/launch_realsense_with_retry.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour el bachari',
    maintainer_email='n.elbachari@gmail.com',
    description='Camera intrinsics displacement check, independent of the hand-eye calibration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intrinsics_test = calib_intrinsics_test.intrinsics_test_node:main',
            'grid_intrinsics_check = calib_intrinsics_test.grid_intrinsics_check_node:main',
        ],
    },
)
