from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tb6612_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mkouda',
    maintainer_email='mkouda@example.com',
    description='ROS2 driver for TB6612 motor driver with Raspberry Pi Pico W',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb6612_driver_node = tb6612_driver.tb6612_driver_node:main',
        ],
    },
)
