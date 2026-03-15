#!/usr/bin/env python3
# rotate_optimize_node.py - 360度回転してAMCL共分散が最小の位置を記録し、その位置でinitial_poseを設定する

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import time
import math
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import numpy as np


class RotateAndOptimizeNode(Node):
    """回転しながら最適な位置を検出するノード"""
    
    def __init__(self):
        super().__init__('rotate_and_optimize')
        
        # 回転パラメータ
        self.declare_parameter('angular_speed', 0.4)        # rad/s - 少し速めに
        self.declare_parameter('rotation_angle', 360.0)     # 度
        self.declare_parameter('completion_wait', 1.5)      # 完了後の待機時間 - 少し長く
        
        # スキャントピック品質向上用パラメータ
        self.declare_parameter('data_collection_rate', 10.0)  # データ収集レート (Hz)
        self.declare_parameter('min_samples', 20)            # 最低サンプル数
        
        # QoSプロファイル設定 - より詳細に設定
        # cmd_vel用QoS - 確実に届けるために信頼性重視
        cmd_vel_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # initial_pose用QoS - 確実に届けるために信頼性重視
        pose_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # amcl_pose用QoS - センサーデータなのでBEST_EFFORT
        amcl_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # トピック設定 - 適切なQoSプロファイルを使用
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', pose_qos
        )
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, amcl_qos
        )
        
        # データ収集用
        self.poses = []
        self.covariances = []
        self.angles = []
        self.start_time = None
        self.best_pose = None
        self.min_covariance = float('inf')
        self.total_angle = 0.0
        self.last_time = None
        self.data_lock = threading.Lock()
        
        # データ収集レート管理用
        self.last_collection_time = 0
        self.collection_interval = 1.0 / self.get_parameter('data_collection_rate').value
        self.min_samples = self.get_parameter('min_samples').value
        
        # パラメータの取得
        self.angular_speed = self.get_parameter('angular_speed').value  # rad/s
        self.rotation_angle = math.radians(self.get_parameter('rotation_angle').value)
        self.completion_wait = self.get_parameter('completion_wait').value
        
        # ログ出力
        self.get_logger().info(
            f'【QoS強化版】回転して最適位置を検出します:\n'
            f'  - 角速度: {self.angular_speed} rad/s ({math.degrees(self.angular_speed):.1f}度/秒)\n'
            f'  - 回転角度: {math.degrees(self.rotation_angle):.1f}度\n'
            f'  - データ収集レート: {self.get_parameter("data_collection_rate").value}Hz\n'
            f'  - 最低サンプル数: {self.min_samples}個\n'
        )
    
    def amcl_pose_callback(self, msg):
        """AMCLからのポーズ情報を受信して記録"""
        # レート制限（高頻度のコールバックでデータが多すぎるのを防ぐ）
        current_time = time.time()
        if current_time - self.last_collection_time < self.collection_interval:
            return
        
        self.last_collection_time = current_time
        
        with self.data_lock:
            # 初回のコールバックで時間を初期化
            if self.start_time is None:
                self.start_time = time.time()
                self.last_time = self.start_time
                self.get_logger().info('AMCL位置データの収集を開始します')
                return  # 初回は記録しない
            
            # 経過時間と回転角を計算
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            
            # 現在の回転角度を蓄積（角速度✕時間）
            angle_increment = self.angular_speed * dt
            self.total_angle += angle_increment
            
            # yaw方向の共分散を取得
            yaw_covariance = msg.pose.covariance[35]
            
            # 位置と共分散を記録
            self.poses.append(msg.pose.pose)
            self.covariances.append(yaw_covariance)
            self.angles.append(self.total_angle)
            
            # 最小共分散を持つポーズを更新
            if yaw_covariance < self.min_covariance:
                self.min_covariance = yaw_covariance
                self.best_pose = msg.pose
                
                # 四元数からオイラー角（ヨー角）を計算
                q = msg.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                
                self.get_logger().info(
                    f'新しい最適ポーズを検出: 共分散 = {yaw_covariance:.5f} rad² '
                    f'(約{math.degrees(math.sqrt(yaw_covariance)):.1f}度の精度)\n'
                    f'位置: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, '
                    f'θ={math.degrees(yaw):.1f}度'
                )
    
    def perform_rotation(self):
        """回転しながらデータを収集し、最適な位置を特定"""
        try:
            # 停止状態から開始
            self.stop_robot()
            time.sleep(0.5)
            
            # 回転開始
            self.get_logger().info(f'回転を開始します: 360度')
            twist_msg = Twist()
            twist_msg.angular.z = self.angular_speed
            
            # 回転時間の計算 (時間 = 角度÷角速度)
            rotation_time = abs(self.rotation_angle / self.angular_speed)
            
            self.get_logger().info(f'予定回転時間: {rotation_time:.2f}秒')
            start_time = time.time()
            end_time = start_time + rotation_time
            
            # 経過表示用
            progress_interval = rotation_time / 8
            next_progress = start_time + progress_interval
            progress_count = 1
            
            # 回転実行
            while time.time() < end_time:
                # 進捗表示
                if time.time() >= next_progress:
                    progress_percent = (progress_count / 8.0) * 100.0
                    self.get_logger().info(f'回転進捗: {progress_percent:.1f}% 完了')
                    next_progress += progress_interval
                    progress_count += 1
                
                self.cmd_vel_pub.publish(twist_msg)
                time.sleep(0.05)  # 20Hzでコマンド送信 (より滑らかに)
            
            # 停止
            self.stop_robot()
            self.get_logger().info('回転完了、データ分析中...')
            
            # サンプル数の確認
            with self.data_lock:
                sample_count = len(self.covariances)
                self.get_logger().info(f'収集したサンプル数: {sample_count}個')
                
                if sample_count < self.min_samples:
                    self.get_logger().warn(
                        f'収集サンプル数が最低要件({self.min_samples})を下回っています。'
                        f'AMCL位置推定サービスが正しく動作しているか確認してください。'
                    )
            
            # 少し待機してAMCLの最終計算を待つ
            time.sleep(1.0)
            
            # 最適な位置を適用
            result = self.apply_best_pose()
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'回転中にエラーが発生しました: {str(e)}')
            self.stop_robot()
            return False
    
    def apply_best_pose(self):
        """最適な位置をinitial_poseとして発行"""
        with self.data_lock:
            if self.best_pose is None:
                self.get_logger().warn('最適な位置が見つかりませんでした')
                return False
            
            # 四元数からオイラー角（ヨー角）を計算
            q = self.best_pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            self.get_logger().info(
                f'最適な位置を適用します: 共分散 = {self.min_covariance:.5f} rad² '
                f'(約{math.degrees(math.sqrt(self.min_covariance)):.1f}度の精度)\n'
                f'位置: x={self.best_pose.position.x:.3f}, y={self.best_pose.position.y:.3f}, '
                f'θ={math.degrees(yaw):.2f}度'
            )
            
            # initial_poseメッセージを作成
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose = self.best_pose  # 最適な位置を使用
            
            # 共分散行列の設定 - 位置推定の信頼度を高く
            pose_msg.pose.covariance = [0.0] * 36  # すべてゼロで初期化
            pose_msg.pose.covariance[0] = 0.1   # x
            pose_msg.pose.covariance[7] = 0.1   # y
            pose_msg.pose.covariance[35] = 0.05 # theta (低い値=高い信頼度)
            
            # 少し待機してから発行（システムの安定化のため）
            time.sleep(self.completion_wait)
            
            # 発行を複数回行って確実に適用されるようにする
            self.get_logger().info('初期位置を更新します (5回送信)')
            for i in range(5):  # 5回に増やす
                pose_msg.header.stamp = self.get_clock().now().to_msg()  # タイムスタンプを更新
                self.initial_pose_pub.publish(pose_msg)
                self.get_logger().debug(f'initial_pose送信 {i+1}/5')
                time.sleep(0.3)  # 間隔を長めに取る
            
            self.get_logger().info('初期位置を更新しました')
            return True
    
    def stop_robot(self):
        """ロボットを停止させる"""
        stop_msg = Twist()
        # すべての速度をゼロに
        for _ in range(5):  # 5回に増やして確実に停止
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.1)
        self.get_logger().debug('ロボットを停止しました')


def main(args=None):
    rclpy.init(args=args)
    node = RotateAndOptimizeNode()
    try:
        # 開始前に少し待機
        node.get_logger().info('システムの準備中...')
        time.sleep(2.0)
        
        # 回転して最適なポーズを見つける
        success = node.perform_rotation()
        if success:
            node.get_logger().info('位置最適化が完了しました ✓')
        else:
            node.get_logger().error('位置最適化に失敗しました ✗')
            
    except Exception as e:
        node.get_logger().error(f'エラーが発生しました: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
