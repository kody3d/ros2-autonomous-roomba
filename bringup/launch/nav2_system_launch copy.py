from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    bringup_share = FindPackageShare("bringup").find("bringup")
    ydlidar_share = FindPackageShare("ydlidar_ros2_driver").find("ydlidar_ros2_driver")
    rosbridge_share = FindPackageShare("rosbridge_server").find("rosbridge_server")
    create_bringup_share = FindPackageShare("create_bringup").find("create_bringup")

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_share, "launch", "tmini.launch.py")
        )
    )

    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "nav2launch.py")
                ),
                launch_arguments={"use_rviz": "false"}.items()
            )
        ]
    )

    rosbridge_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(rosbridge_share, "launch", "rosbridge_websocket_launch.xml")
                )
            )
        ]
    )

    create2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(create_bringup_share, "launch", "create_2.launch")
                ),
                launch_arguments={
                    "dev": "/dev/ft232",
                    "baud": "115200"
                }.items()
            )
        ]
    )

    # 古いRVizトピックを新しいNav2トピックにリレー
    topic_relay_goal = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/move_base_simple/goal", "/goal_pose"],
        output="screen"
    )


    return LaunchDescription([
        ydlidar_launch,
        nav2_launch,
        rosbridge_launch,
        create2_launch,
        topic_relay_goal,      # トピックリレー追加
    ])
