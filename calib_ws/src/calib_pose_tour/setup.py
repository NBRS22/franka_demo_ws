from setuptools import find_packages, setup

package_name = 'calib_pose_tour'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour el bachari',
    maintainer_email='n.elbachari@gmail.com',
    description='Moves fp3_hand through a pose tour for hand-eye calibration sampling',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calib_pose_tour = calib_pose_tour.calibration_pose_tour_node:main',
        ],
    },
)
