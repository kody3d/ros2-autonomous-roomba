import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    bringup_dir = get_package_share_directory('bringup')
    bno055_dir = get_package_share_directory('bno055')
    cartographer_config_dir = os.path.join(bringup_dir, 'config')
    # configuration_basename = 'noimu_lds_2d.lua'
    configuration_basename = 'detail_imu_2d.lua'
    # configuration_basename = '619detail.lua'
    

    use_sim_time = False
    resolution = '0.025'
    publish_period_sec = '1.0'

    # BNO055 IMUノード（内部通信のみ、外部公開なし）
    # BNO055パラメータファイルのパス　
    # paramsでなくconfig
    bno055_params_path = os.path.join(bno055_dir, 'config', 'bno055_params.yaml')
    # bno055_params_path = '/home/mkouda/create_ws/src/bno055/bno055/params/bno055_params.yaml'
    
    # BNO055 IMUノード（パラメータファイル使用）
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[bno055_params_path]
    )
        
#         parameters=[{
#             'connection_type': 'i2c',
#             'i2c_bus': 1,
#             'i2c_addr': 0x29,
#             'frame_id': 'imu_link',
#             'ros_topic_prefix': '',  # プレフィックスなしで内部通信を最小化
#             # 'data_query_frequency': 10,  # 頻度を下げてDDS負荷軽減
#             'calib_status_frequency': 1.0,
#             'placement_axis_remap': 'P0'  # P0, P1, P2, P3, P4, P5, P6, P7を試す
# #     P0: デフォルト
# # P1: X-Y軸入れ替え
# # P2: X軸反転
# # P3: Y軸反転
# # P4: Z軸反転
# # P5: XY軸入れ替え + X軸反転
# # P6: XY軸入れ替え + Y軸反転
# # P7: 全軸反転
#         }]
    

    # Static Transform Publisher (base_link -> imu_link) - IMUは同じ位置に配置
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    # )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=[
            '0',    # x
            '0',    # y
            '0',    # z - base_footprintと同じ位置
            '0',    # qx
            '0',    # qy
            '0',    # qz
            '1',    # qw - 回転なし
            'base_link',  # base_link の代わりにbase_footprintを使用
            'imu_link'
        ]
    )


    # Cartographerノード（内部IMU通信のみ）
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'configuration_directory': cartographer_config_dir,
            'configuration_basename': configuration_basename,
        }],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu'),  # プレフィックスなしのIMUトピック
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

    # rviz2_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=[["-d"], [rviz2_config]]
    # )

    # LaunchDescriptionに追加
    ld = LaunchDescription()
    ld.add_action(bno055_node)
    ld.add_action(static_tf_node)
    # ld.add_action(map_correction_tf) 
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    # ld.add_action(rviz2_node)

    return ld

