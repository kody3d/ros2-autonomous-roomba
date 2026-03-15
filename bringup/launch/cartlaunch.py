import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

import math


lidar_yaw_deg = -1.1


def yaw_to_quaternion(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    qz = math.sin(yaw_rad / 2)
    qw = math.cos(yaw_rad / 2)
    return qz, qw


def generate_launch_description():
    
    bringup_dir = get_package_share_directory('bringup')
    cartographer_config_dir = os.path.join(bringup_dir, 'config')
    configuration_basename = 'noimu_lds_2d.lua'
    # configuration_basename = 'detail_noimu_2d.lua'
    # configuration_basename = 'noimu_lds_2d5.lua'
 
    use_sim_time = False
    resolution = '0.02'
    publish_period_sec = '1.0'

    # Cartographerノード
    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],

            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu'),
                ('odom', '/odom'),  # ← オドメトリトピックを追加
            ]
    )

    # Occupancy Gridノード
    occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    # RVizノード
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
        arguments=[["-d"], [rviz2_config]]
    )

    # 地図を読み込むための map_server ノードを追加
    # map_yaml_file = "/home/mkouda/map.yaml"  # 地図ファイルのパス（適宜変更）
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=['--ros-args', '--param', f'yaml_filename:={map_yaml_file}']
    # )

     # ここでクォータニオンを計算
    qz, qw = yaw_to_quaternion(lidar_yaw_deg)

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        # arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
        # arguments=[
        #     '0', '0', '0.1', '0', '0', '3.14159', '0', 'base_link',
        #     'laser_frame'
        # ],
        arguments=[
            '0.0    s3',
            '0',
            '0.1',  # XYZ位置
            '0',
            '0',
            # '0',
            # '1',  # クォータニオン [qx, qy, qz, qw]
            str(qz),
            str(qw),
            #  '0.3420', '0.9397', 
            'base_link',       # ← base_linkからbase_footprintに変更. 6/19base_link戻す
            'laser_frame'  # フレーム名
        ])


    # LaunchDescriptionに追加
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    # ld.add_action(tf2_node)
    
    # ld.add_action(rviz2_node)
    # ld.add_action(map_server_node)  # map_serverノードを追加


    return ld

