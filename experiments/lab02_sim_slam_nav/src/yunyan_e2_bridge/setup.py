"""Setup file for yunyan_e2_bridge package."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'yunyan_e2_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.*')
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.*')
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Auto_Driving',
    maintainer_email='yunyan-lab@users.noreply.github.com',
    description=(
        'ROS 2 Humble bridge package for PadBot E2 mobile base '
        'via HTTP RESTful API'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'e2_bridge = yunyan_e2_bridge.e2_bridge_node:main',
            'e2_mock_server = yunyan_e2_bridge.e2_mock_server:main',
            'e2_probe = yunyan_e2_bridge.e2_probe:main',
            'e2_waypoint_patrol = yunyan_e2_bridge.e2_waypoint_patrol:main',
        ],
    },
)
