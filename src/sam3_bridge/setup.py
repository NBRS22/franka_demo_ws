from setuptools import find_packages, setup

package_name = 'sam3_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sam3_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour.el.bachari',
    maintainer_email='nour.el.bachari@accenture.com',
    description='Bridge for communicating with the SAM3 system',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sam3_bridge_node = sam3_bridge.sam3_bridge_node:main',
            'visualize_segmentation_node = sam3_bridge.visualize_segmentation_node:main',
        ],
    },
)
