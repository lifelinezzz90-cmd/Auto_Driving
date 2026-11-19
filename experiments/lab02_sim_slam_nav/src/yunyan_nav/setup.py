"""Setup file for yunyan_nav package."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'yunyan_nav'

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
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.*')
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Auto_Driving',
    maintainer_email='yunyan-lab@users.noreply.github.com',
    description='SLAM Toolbox mapping and Nav2 configuration package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apf_avoidance = yunyan_nav.apf_avoidance:main',
        ],
    },
)
