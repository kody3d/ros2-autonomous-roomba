from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen'
        ),
        Node(
            package='joy_translate',
            executable='joy_translate_node',
            name='joy_translate_node',
            output='screen'
        ),
    ])

