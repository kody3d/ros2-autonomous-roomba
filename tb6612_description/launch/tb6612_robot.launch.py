#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

import os

# sudo apt install ros-humble-joint-state-publisher

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('tb6612_description')
    
    # Path to URDF file
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'tb6612_robot.urdf.xacro')
    
    # Launch configuration variables
    urdf_file = LaunchConfiguration('urdf_file')
    
    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf_path,
        description='Path to robot URDF file'
    )
    
    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]), value_type=str)
        }]
    )
    
    # Joint State Publisher node (for static joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        urdf_file_arg,
        robot_state_publisher,
        joint_state_publisher
    ])
