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
    
    # 基本的なTF変換ノード（QoS設定強化）
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'qos_overrides': {
                '/tf_static': {
                    'qos_profile': 'system_default',
                    'depth': 10,
                    'durability': 'transient_local',
                    'reliability': 'reliable'
                }
            }
        }],
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'qos_overrides': {
                '/tf_static': {
                    'qos_profile': 'system_default',
                    'depth': 10,
                    'durability': 'transient_local',
                    'reliability': 'reliable'
                }
            }
        }],
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'base_footprint']
    )
    
    # laser_frame用のTF変換も追加（Message Filter問題対応）
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'qos_overrides': {
                '/tf_static': {
                    'qos_profile': 'system_default',
                    'depth': 10,
                    'durability': 'transient_local',
                    'reliability': 'reliable'
                }
            }
        }],
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
    )

    # 絶対パスを使用して指定
    amcl_params_file = os.path.join(bringup_dir, 'params', 'amcl_params.yaml')
    nav2_params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')

    # AMCLノードの設定（パスを修正）
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/localization_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,  # LaunchConfigurationを使用
            'map': map_yaml_file,
            'params_file': amcl_params_file,  # 変数を使用
            'use_lifecycle_mgr': 'false',
            'autostart': 'true'
        }.items()
    )
    
    # Nav2ナビゲーション機能
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,  # LaunchConfigurationを使用
            'params_file': nav2_params_file,  # 変数を使用
            'autostart': 'true',
            'use_lifecycle_mgr': 'true'  # ライフサイクルマネージャーを有効化
        }.items()
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
        arguments=['-d', rviz2_config],
        parameters=[{'use_sim_time': use_sim_time}],  # LaunchConfigurationを使用
        remappings=[('/move_base_simple/goal','/goal_pose')],
        condition=IfCondition(use_rviz)
    )
    
    # 初期位置パブリッシャー
    initial_pose_node = ExecuteProcess(
        cmd=['python3', os.path.join(bringup_dir, 'launch', 'initial_pose_publisher.py')],
        name='initial_pose_publisher',
        output='screen'
    )


        # 古いRVizトピックを新しいNav2トピックにリレー
    topic_relay_goal = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/move_base_simple/goal", "/goal_pose"],
        output="screen"
    )

    
    # LaunchDescriptionに追加
    ld = LaunchDescription()
    
    # パラメータ宣言
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    # 基本的なTF変換を優先設定
    ld.add_action(static_tf)
    # ld.add_action(base_footprint_tf)
    # ld.add_action(laser_tf)
    

    ld.add_action(topic_relay_goal)


    # コンポーネントの起動順序
    ld.add_action(amcl_launch)
    ld.add_action(TimerAction(
    period=5.0,  # 30.0から25.0に調整
    actions=[rviz2_node]
))


    ld.add_action(TimerAction(
        period=0.0, 
        actions=[nav2_launch]
    ))
    
    # 時間を長めに設定
    # ld.add_action(TimerAction(
    #     period=5.0,
    #     actions=[initial_pose_node]
    # ))
    

    return ld

