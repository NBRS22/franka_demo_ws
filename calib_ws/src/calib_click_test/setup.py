from setuptools import find_packages, setup

package_name = 'calib_click_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/click_test.launch.py']),
        ('share/' + package_name + '/scripts', ['scripts/launch_realsense_with_retry.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour el bachari',
    maintainer_email='n.elbachari@gmail.com',
    description='Click a point in the point cloud, the arm moves fp3_hand_tcp there',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'click_test = calib_click_test.click_to_point_node:main',
        ],
    },
)
