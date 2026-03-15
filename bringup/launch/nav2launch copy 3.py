import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSDurabilityPolicy

def generate_launch_description():
    
    bringup_dir = get_package_share_directory('bringup')
    
    # use_sim_timeをLaunchConfigurationとして定義（bool型のエラー対策）
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # use_rvizパラメータの追加
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )
 
    # パラメータ定義
    map_yaml_file = "/home/mkouda/map.yaml"
    
    # 基本的なTF変換ノード
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'transform_tolerance': '1.0'
        }],
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        output='screen',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'base_footprint']
    )
    
    # 初期位置パブリッシャー
    initial_pose_script = os.path.join(
        bringup_dir,
        'launch',
        'initial_pose_publisher.py'
    )
    
    initial_pose_node = ExecuteProcess(
        cmd=['python3', initial_pose_script],
        name='initial_pose_publisher',
        output='screen'
    )
    
    # 初期回転運動ノード
    initial_rotation_script = os.path.join(
        bringup_dir,
        'launch',
        'initial_rotation.py'
    )
    
    initial_rotation_node = ExecuteProcess(
        cmd=['python3', initial_rotation_script],
        name='initial_rotation',
        output='screen'
    )
    
    # RViz2ノード
    rviz2_config = os.path.join(
        get_package_share_directory('bringup'),
        'rviz',
        'config.rviz'
    )
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_config]],
        parameters=[{
            'use_sim_time': use_sim_time,
            'qos_reliability': 'best_effort'
        }],
        remappings=[('/move_base_simple/goal','/goal_pose')],
        condition=IfCondition(use_rviz)
    )
    
    # AMCLノードを構築（TimerActionの外で定義）
    amcl_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'), 
            '/launch/localization_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'amcl.update_min_d': '0.05',
            'amcl.update_min_a': '0.03',
            'amcl.resample_interval': '1',
            'amcl.laser_max_range': '3.5',
            'amcl.laser_model_type': 'likelihood_field',
            'amcl.min_particles': '500',
            'amcl.max_particles': '3000',
            'map': map_yaml_file,
            'scan_topic': '/scan',
            'global_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'transform_tolerance': '2.0',
            'tf_broadcast': 'true',
        }.items()
    )
    
    # Nav2ノードを構築（TimerActionの外で定義）
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'controller_server.controller_frequency': '5.0',
            'collision_monitor.observation_sources': 'scan',
            'collision_monitor.scan.topic': '/scan',
            'collision_monitor.scan.data_type': 'LaserScan',
            'collision_monitor.scan.qos_reliability': 'best_effort',
            'collision_monitor.scan.qos_durability': 'volatile',
            'local_costmap.local_costmap.scan.qos_reliability': 'best_effort',
            'local_costmap.local_costmap.scan.qos_durability': 'volatile',
            'controller_server.FollowPath.scan.qos_reliability': 'best_effort',
            'controller_server.FollowPath.scan.qos_durability': 'volatile',
        }.items()
    )
    
    # LaunchDescriptionに追加
    ld = LaunchDescription()
    
    # パラメータ宣言
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # 基本的なTF変換を優先設定
    ld.add_action(static_tf)
    ld.add_action(base_footprint_tf)
    
    # ログメッセージの追加
    ld.add_action(LogInfo(msg="Nav2起動シーケンスを開始します"))
    
    # AMCLを起動
    ld.add_action(amcl_node)
    ld.add_action(LogInfo(msg="AMCL起動完了、5秒後に初期位置設定を開始します"))
    
    # AMCL起動から5秒後に初期位置を設定
    ld.add_action(TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="初期位置設定を開始します"),
            initial_pose_node
        ]
    ))
    
    # 初期位置設定から3秒後に回転運動を実行
    ld.add_action(TimerAction(
        period=8.0,  # 5秒 + 3秒
        actions=[
            LogInfo(msg="初期回転運動を開始します"),
            initial_rotation_node
        ]
    ))
    
    # 回転運動から5秒後にナビゲーションとRVizを起動
    ld.add_action(TimerAction(
        period=13.0,  # 5秒 + 3秒 + 5秒
        actions=[
            LogInfo(msg="ナビゲーションとRVizを起動します"),
            # 注意: ここでIncludeLaunchDescriptionインスタンスをTimerAction内で直接構築しない
            # 代わりに、あらかじめ構築されたインスタンスを使用
            nav2_launch,
            rviz2_node
        ]
    ))
    
    return ld

