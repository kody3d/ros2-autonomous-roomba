#!/usr/bin/env python3
"""
optimize_localization.py - 
360度回転してAMCL共分散が最小の位置を記録し、その位置でinitial_poseを設定する。
初期位置設定メッセージが確実に届くよう、ノードとして長期間動作し、複数回再送信します。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node as LaunchNode
from launch.substitutions import LaunchConfiguration
import rclpy
from rclpy.node import Node as RclpyNode
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import time
import math
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy


class RotateAndOptimizeNode(RclpyNode):
    """回転しながら最適な位置を検出し、初期位置を更新するノード"""

    def __init__(self):
        super().__init__('rotate_and_optimize')
        # 回転パラメータの宣言
        self.declare_parameter('angular_speed', 0.4)        # rad/s
        self.declare_parameter('rotation_angle', 360.0)       # 度
        self.declare_parameter('completion_wait', 1.5)        # 初期位置発行前待機時間
        self.declare_parameter('data_collection_rate', 10.0)  # Hz
        self.declare_parameter('min_samples', 20)             # 最低サンプル数

        # QoSプロファイル設定
        cmd_vel_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        pose_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        amcl_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # トピック設定
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', pose_qos)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, amcl_qos)

        # データ収集用変数
        self.poses = []
        self.covariances = []
        self.angles = []
        self.start_time = None
        self.best_pose = None
        self.min_covariance = float('inf')
        self.total_angle = 0.0
        self.last_time = None
        self.data_lock = threading.Lock()

        # データ収集レート管理
        self.last_collection_time = 0
        self.collection_interval = 1.0 / self.get_parameter('data_collection_rate').value
        self.min_samples = self.get_parameter('min_samples').value

        # パラメータ取得
        self.angular_speed = self.get_parameter('angular_speed').value  # rad/s
        self.rotation_angle = math.radians(self.get_parameter('rotation_angle').value)
        self.completion_wait = self.get_parameter('completion_wait').value

        self.get_logger().info(
            f'【QoS強化版】回転して最適位置を検出します:\n'
            f'  - 角速度: {self.angular_speed} rad/s ({math.degrees(self.angular_speed):.1f}度/秒)\n'
            f'  - 回転角度: {math.degrees(self.rotation_angle):.1f}度\n'
            f'  - データ収集レート: {self.get_parameter("data_collection_rate").value}Hz\n'
            f'  - 最低サンプル数: {self.min_samples}個'
        )

    def amcl_pose_callback(self, msg):
        """AMCLからのポーズ情報を受信して記録"""
        current_time = time.time()
        if current_time - self.last_collection_time < self.collection_interval:
            return
        self.last_collection_time = current_time

        with self.data_lock:
            # 初回はタイミング合わせのため記録せずに初期化
            if self.start_time is None:
                self.start_time = time.time()
                self.last_time = self.start_time
                self.get_logger().info('AMCL位置データの収集を開始します')
                return

            dt = current_time - self.last_time
            self.last_time = current_time
            angle_increment = self.angular_speed * dt
            self.total_angle += angle_increment
            yaw_covariance = msg.pose.covariance[35]

            self.poses.append(msg.pose.pose)
            self.covariances.append(yaw_covariance)
            self.angles.append(self.total_angle)

            if yaw_covariance < self.min_covariance:
                self.min_covariance = yaw_covariance
                self.best_pose = msg.pose
                q = msg.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.get_logger().info(
                    f'新しい最適ポーズを検出: 共分散 = {yaw_covariance:.5f} rad² '
                    f'(約{math.degrees(math.sqrt(yaw_covariance)):.1f}度の精度), '
                    f'位置: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, '
                    f'θ={math.degrees(yaw):.1f}度'
                )

    def perform_rotation(self):
        """回転しながらデータを収集し、最適な位置を特定する"""
        try:
            self.stop_robot()
            time.sleep(0.5)
            self.get_logger().info('回転を開始します: 360度')
            twist_msg = Twist()
            twist_msg.angular.z = self.angular_speed
            rotation_time = abs(self.rotation_angle / self.angular_speed)
            self.get_logger().info(f'予定回転時間: {rotation_time:.2f}秒')
            start_time = time.time()
            end_time = start_time + rotation_time

            progress_interval = rotation_time / 8
            next_progress = start_time + progress_interval
            progress_count = 1

            while time.time() < end_time:
                if time.time() >= next_progress:
                    progress_percent = (progress_count / 8.0) * 100.0
                    self.get_logger().info(f'回転進捗: {progress_percent:.1f}% 完了')
                    next_progress += progress_interval
                    progress_count += 1
                self.cmd_vel_pub.publish(twist_msg)
                time.sleep(0.05)
            
            self.stop_robot()
            self.get_logger().info('回転完了、データ分析中...')
            with self.data_lock:
                sample_count = len(self.covariances)
                self.get_logger().info(f'収集したサンプル数: {sample_count}個')
                if sample_count < self.min_samples:
                    self.get_logger().warn(
                        f'収集サンプル数が最低要件({self.min_samples})を下回っています。'
                        'AMCL位置推定サービスを確認してください。'
                    )
            time.sleep(1.0)
            return self.apply_best_pose()
        except Exception as e:
            self.get_logger().error(f'回転中にエラーが発生しました: {str(e)}')
            self.stop_robot()
            return False

    def apply_best_pose(self):
        """最適な位置をinitial_poseとして発行し、複数回再送信する"""
        with self.data_lock:
            if self.best_pose is None:
                self.get_logger().warn('最適な位置が見つかりませんでした')
                return False

            q = self.best_pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.get_logger().info(
                f'最適な位置を適用します: 共分散 = {self.min_covariance:.5f} rad² '
                f'(約{math.degrees(math.sqrt(self.min_covariance)):.1f}度の精度), '
                f'位置: x={self.best_pose.position.x:.3f}, y={self.best_pose.position.y:.3f}, '
                f'θ={math.degrees(yaw):.2f}度'
            )

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose = self.best_pose  # 最適な位置を使用
            pose_msg.pose.covariance = [0.0] * 36
            pose_msg.pose.covariance[0] = 0.1
            pose_msg.pose.covariance[7] = 0.1
            pose_msg.pose.covariance[35] = 0.05

            self.get_logger().info('初期位置の再送信を開始します (約7.5秒間、15回送信)')
            time.sleep(self.completion_wait)
            for i in range(15):  # 15回送信 (約7.5秒)
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                self.initial_pose_pub.publish(pose_msg)
                self.get_logger().debug(f'initial_pose送信 {i+1}/15')
                time.sleep(0.5)
            self.get_logger().info('初期位置の再送信が完了しました')
            return True

    def stop_robot(self):
        """ロボットを停止させる"""
        stop_msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.1)
        self.get_logger().debug('ロボットを停止しました')


def generate_launch_description():
    """
    optimize_localization.py をノードとして起動するためのLaunchDescriptionです。
    ExecuteProcessではなく、Nodeとして起動するため、タイミングやライフサイクルが一貫して管理されます。
    """
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time if true'
    )

    optimize_node = LaunchNode(
        package='bringup',
        executable='optimize_localization.py',
        name='optimize_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(optimize_node)
    return ld


def main():
    rclpy.init()
    node = RotateAndOptimizeNode()
    try:
        node.get_logger().info('Nav2とAMCLの起動を待機中...')
        time.sleep(5.0)
        success = node.perform_rotation()
        if success:
            node.get_logger().info('位置最適化が完了しました ✓')
        else:
            node.get_logger().error('位置最適化に失敗しました ✗')
        # 初期位置メッセージの伝達を保証するため、追加で待機
        time.sleep(2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()