#!/usr/bin/env python3
"""
TB6612 Driver Launch File

TB6612モータードライバノードをパラメータ付きで起動する
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyS0',
            description='Serial port for TB6612 communication'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='230400',
            description='Serial baud rate'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Base frame ID for robot'
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odometry frame ID'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Whether to publish TF transforms'
        ),
        DeclareLaunchArgument(
            'max_linear_vel',
            default_value='1.0',
            description='Maximum linear velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_vel',
            default_value='3.14',
            description='Maximum angular velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'cmd_vel_timeout',
            default_value='1.0',
            description='Command velocity timeout (seconds)'
        ),
        DeclareLaunchArgument(
            'wheel_base',
            default_value='0.16',
            description='Distance between wheels (m)'
        ),
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.033',
            description='Wheel radius (m)'
        ),

        # TB6612 Driver Node
        Node(
            package='tb6612_driver',
            executable='tb6612_driver_node',
            name='tb6612_driver',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'max_linear_vel': LaunchConfiguration('max_linear_vel'),
                'max_angular_vel': LaunchConfiguration('max_angular_vel'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
                'wheel_base': LaunchConfiguration('wheel_base'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'serial_timeout': 0.1,
                'diagnostic_period': 1.0,
            }],
            remappings=[
                # 必要に応じてトピック名をリマップ
                # ('cmd_vel', '/robot/cmd_vel'),
                # ('odom', '/robot/odom'),
            ]
        ),
    ])
