# humble
# raspi4でnav2まで動かしていいます。
# 別PCで rviz可視化 initialpose goalposeを設定します。
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Nav2関連のパッケージパス
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    create3_nav_dir = get_package_share_directory('create3_nav')

    # パラメータファイル
    nav2_params_file = os.path.join(create3_nav_dir, 'params',
                                    'create3_nav_params.yaml')

    # 地図ファイル - 絶対パスで指定
    map_dir = os.path.join(create3_nav_dir, 'maps')
    map_file = os.path.join(map_dir, 'map.yaml')
    
    # map_fileが存在するか確認
    if not os.path.isfile(map_file):
        map_file_alternatives = [f for f in os.listdir(map_dir) if f.endswith('.yaml')]
        if map_file_alternatives:
            map_file = os.path.join(map_dir, map_file_alternatives[0])
            print(f"指定のmap.yamlが見つからないため、代替として {map_file} を使用します")
        else:
            print(f"警告: {map_dir} に有効なマップファイルが見つかりません")

    # URDFファイルをロード
    urdf_file = os.path.join(create3_nav_dir, 'urdf', 'create_1.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ローンチ設定
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    map_yaml_file = LaunchConfiguration('map')

    # ローンチ引数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='シミュレーション時間を使うかどうか')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Nav2スタックを自動的に起動するかどうか')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='地図ファイルへの絶対パス')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Nav2のパラメータファイルへの絶対パス')

    # ロボットステートパブリッシャー
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }])

    # 必要なTF変換の提供
    # base_footprint → base_link
    footprint_to_base_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint_to_base_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'] # 新しい形式
    )
    
    # マップサーバーを独立して起動（常時送信設定）
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
            'topic_name': 'map',
            'frame_id': 'map',
            # マップを定期的に送信する設定
            'always_send_full_costmap': True,
            # マップ更新の頻度（Hz）- 1Hzが安定して動作する値
            'map_update_rate': 1.0
        }])
    
    # ローカリゼーション（AMCL）を独立して起動
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': nav2_params_file,
            'use_lifecycle_mgr': 'true',
            'autostart': autostart,
        }.items())
    
    # ナビゲーション機能だけを含む（マップサーバーとローカライゼーションを除く）
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'use_lifecycle_mgr': 'false',  # 上記で既に起動しているため
            'autostart': autostart,
        }.items())

    # 段階的な起動シーケンス（nav2launchパターンを適用）
    delayed_localization_cmd = TimerAction(
        period=1.0,
        actions=[localization_cmd]
    )
    
    delayed_navigation_cmd = TimerAction(
        period=3.0,
        actions=[navigation_cmd]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        
        # 最初に起動：基本コンポーネント
        robot_state_publisher_cmd,
        footprint_to_base_tf_cmd,
        
        # マップサーバーを独立して起動（常時送信設定）
        map_server_cmd,
        
        # 段階的に遅延してコンポーネントを起動
        delayed_localization_cmd,
        delayed_navigation_cmd,
    ])