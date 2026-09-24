from setuptools import find_packages, setup

package_name = 'calib_sample_guard'

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
    description='Live reprojection-error/tilt guardrail for hand-eye calibration sampling',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample_guard = calib_sample_guard.sample_guard_node:main',
        ],
    },
)
