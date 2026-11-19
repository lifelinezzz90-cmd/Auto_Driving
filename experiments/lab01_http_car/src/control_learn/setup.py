from setuptools import find_packages, setup

package_name = 'control_learn'

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
    maintainer='Auto_Driving',
    maintainer_email='yunyan-lab@users.noreply.github.com',
    description='Debug node that subscribes to and prints cmd_vel for the yunyan car',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_observer = control_learn.cmd_vel_observer:main'
        ],
    },
)
