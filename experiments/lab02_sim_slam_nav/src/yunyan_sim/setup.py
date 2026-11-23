"""Setup file for yunyan_sim package."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'yunyan_sim'

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
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.*')
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.*')
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
    description='Gazebo simulation package for yunyan car',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
