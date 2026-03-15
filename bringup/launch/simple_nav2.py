#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # use_sim_timeをLaunchConfigurationとして定義（bool型のエラー対策）
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # パラメータとパスの設定
    map_yaml_file = '/home/mkouda/map.yaml'
    
    # パッケージディレクトリの取得
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('bringup')
    
    # RViz設定ファイルのパス
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'config.rviz')
    
    # 基本的なTF発行用ノード
    # レーザーフレーム変換の追加
    laser_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
    )

    # base_link -> base_footprintの変換も追加
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        output='screen',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'base_footprint']  # 0.1mの高さ
    )
    
    # Nav2のlocalization_launchをインクルード
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/localization_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            # 追加パラメータ
            'transform_tolerance': '2.0',  # 変換タイムアウト増加
            'tf_broadcast': 'true',  # 明示的に有効化
            'global_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }.items()
    )
    
    # Nav2のnavigation_launchをインクルード
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            'map': map_yaml_file,
            # collision_monitorのパラメータを追加
            'collision_monitor.observation_sources': 'scan',
            'collision_monitor.scan.topic': '/scan',
            'collision_monitor.scan.data_type': 'LaserScan',
            # コントローラー調整
            'controller_server.controller_frequency': '7.0',  # 7Hzに設定
        }.items()
    )
    
    # RViz2ノード - QoS設定を明示的かつ直接的に指定
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--remap', '/scan:=/scan_best_effort'],
        parameters=[{
            'use_sim_time': use_sim_time,
            # RVizの全体的なQoS設定
            'qos_reliability': 'best_effort',
        }],
        output='screen'
    )

    # スキャントピックのリマッピング用のリレーノード
    scan_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        output='screen',
        arguments=['/scan', '/scan_best_effort'],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        # 明示的にQoS設定を指定
        ros_arguments=['--qos-reliability', 'best_effort', 
                      '--qos-durability', 'volatile']
    )
    
    # LaunchDescriptionを作成
    ld = LaunchDescription()
    
    # LaunchConfigurationを追加
    ld.add_action(declare_use_sim_time_cmd)
    
    # TF変換を追加
    ld.add_action(laser_frame_tf)
    ld.add_action(base_footprint_tf)
    
    # マップとロケーライゼーションを起動
    ld.add_action(localization_launch)
    
    # スキャントピックのリレーノードを追加
    ld.add_action(scan_relay_node)
    
    # 少し遅延してナビゲーションを起動
    delayed_nav = TimerAction(
        period=2.0,  # 2秒遅延
        actions=[navigation_launch]
    )
    ld.add_action(delayed_nav)
    
    # さらに遅延してRVizを起動
    delayed_rviz = TimerAction(
        period=3.0,  # 3秒遅延
        actions=[rviz_node]
    )
    ld.add_action(delayed_rviz)
    
    return ld