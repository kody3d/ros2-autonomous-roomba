#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('tb6612_description'), 
                        'launch', 'tb6612_robot.launch.py')
        ])
    )
    
    # TB6612 Driver Node
    tb6612_driver = Node(
        package='tb6612_driver',
        executable='tb6612_driver_node',
        name='tb6612_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyS0',
            'baud_rate': 115200,
            'publish_tf': True,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom'
        }]
    )
    
    # BNO055 IMU Driver (オプション)
    bno055_driver = Node(
        package='bno055',
        executable='bno055',
        name='bno055_imu',
        output='screen',
        parameters=[{
            'frame_id': 'imu_link'
        }]
    )

    return LaunchDescription([
        robot_description_launch,
        tb6612_driver,
        # bno055_driver
    ])
