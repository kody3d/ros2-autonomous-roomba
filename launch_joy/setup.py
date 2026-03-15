from setuptools import find_packages, setup

package_name = 'launch_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
       ('share/' + package_name + '/launch', ['launch/joy_multi_launch.py']),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mkouda',
    maintainer_email='mkouda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
            'launch': [
        'joy_multi_launch = launch.joy_multi_launch:generate_launch_description',
    ],
    },
)
