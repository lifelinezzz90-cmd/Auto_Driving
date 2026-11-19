from glob import glob

from setuptools import find_packages, setup

package_name = 'yunyan_bringup'

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
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Auto_Driving',
    maintainer_email='yunyan-lab@users.noreply.github.com',
    description='ROS 2 cmd_vel to HTTP bridge and keyboard teleop for the yunyan ESP32 car',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros_http_bridge = yunyan_bringup.ros_http_bridge:main',
            'car_teleop = yunyan_bringup.car_teleop:main'
        ],
    },
)
