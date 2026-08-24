from setuptools import find_packages, setup

package_name = 'franka_demo_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/franka_demo.launch.py']),
        ('share/' + package_name + '/scripts', [
            'scripts/wait_for_zmq_health.py',
            'scripts/launch_realsense_with_retry.sh',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour.el.bachari',
    maintainer_email='nour.el.bachari@accenture.com',
    description='Launch files for the Franka demo',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
)
