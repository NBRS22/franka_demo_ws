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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngr',
    maintainer_email='n.elbachari@gmail.com',
    description=(
        'Publie fp3_link0 -> camera_link en static transform a partir '
        "d'une calibration easy_handeye2 et du TF interne de realsense2_camera."
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handeye_tf_publisher = handeye_tf_publisher.publisher_node:main',
        ],
    },
)
