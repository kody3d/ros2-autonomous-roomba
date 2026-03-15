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

    # 古いRVizトピックを新しいNav2トピックにリレー
    topic_relay_goal = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/move_base_simple/goal", "/goal_pose"],
        output="screen"
    )
    create2_launch = TimerAction(
        period=0.0,
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
    rosbridge_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(rosbridge_share, "launch", "rosbridge_websocket_launch.xml")
                )
            )
        ]
    )



    nav2_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "nav2launch.py")
                ),
                launch_arguments={
                    "use_rviz": "false",

                    # マップファイルを指定（重要）
                    "map": "/home/mkouda/map.yaml",  # マップファイルのパスを指定
                
                    # 制御頻度を下げる
                    # 'controller_server.controller_frequency': '10.0',
                    # collision_monitorのパラメータを追加
                    'collision_monitor.observation_sources': 'scan',
                    'collision_monitor.scan.topic': '/scan',
                    'collision_monitor.scan.data_type': 'LaserScan',
                    # YDLidarに合わせたQoS設定（collision_monitor用）
                    'collision_monitor.scan.qos_reliability': 'best_effort',
                    'collision_monitor.scan.qos_durability': 'volatile',
                    # ローカルコストマップのQoS設定
                    'local_costmap.local_costmap.scan.qos_reliability': 'best_effort',
                    'local_costmap.local_costmap.scan.qos_durability': 'volatile',
                    # コントローラーのQoS設定（スキャンを使用する場合）
                    'controller_server.FollowPath.scan.qos_reliability': 'best_effort',
                    'controller_server.FollowPath.scan.qos_durability': 'volatile',
                }.items()
            )
        ]
    )



    return LaunchDescription([
        ydlidar_launch,
        nav2_launch,
        rosbridge_launch,
        create2_launch,
        topic_relay_goal,
    ])
