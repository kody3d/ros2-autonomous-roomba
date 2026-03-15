from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """SLAMシステム全体を起動するlaunchファイル"""
    
    # create_bringupパッケージのcreate_2.launchを含める
    create_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('create_bringup'),
                'launch',
                'create_2.launch'
            ])
        ),
        launch_arguments={
            'dev': '/dev/ft232',
            'baud': '115200'
        }.items()
    )
    
    # ydlidar_ros2_driverパッケージのtmini.launch.pyを含める
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'launch',
                'tmini.launch.py'
            ])
        )
    )
    
    # bringupパッケージのcartlaunch.pyを含める
    cart_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bringup'),
                'launch',
                # 'cartlaunch.py'
                'cartlaunchIMU.py'
            ])
        )
    )
    
    # rosbridge_serverパッケージのrosbridge_websocket_launch.xmlを含める
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            ])
        )
    )
    
    # すべてのlaunchファイルをまとめる
    return LaunchDescription([
        create_launch,
        lidar_launch,
        cart_launch,
        rosbridge_launch
    ])