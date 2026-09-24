from setuptools import find_packages, setup

package_name = 'calib_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/calib_bridge.launch.py',
        ]),
        ('share/' + package_name + '/scripts', ['scripts/launch_realsense_with_retry.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour el bachari',
    maintainer_email='n.elbachari@gmail.com',
    description='Derives the D455 eye-on-base calibration from the D405 eye-in-hand calibration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_calibration_node = calib_bridge.bridge_calibration_node:main',
        ],
    },
)
