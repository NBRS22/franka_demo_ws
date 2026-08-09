from setuptools import find_packages, setup

package_name = 'gemini_er_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour.el.bachari',
    maintainer_email='nour.el.bachari@accenture.com',
    description='Bridge for communicating with the Gemini ER system',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'command_bridge_node = gemini_er_bridge.command_bridge_node:main',
            'camera_bridge_node = gemini_er_bridge.camera_bridge_node:main',
        ],
    },
)
