#!/usr/bin/env python3
"""
covariance_monitor.py - 
AMCLの推定共分散をモニタリングし、姿勢の不確かさの変化を記録します。
複数回の回転の前後でcovariance[35]の値（ヨー角方向の不確かさ）がどのように変化するかを観察します。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import time
import math
import threading
import csv
import os
from datetime import datetime

class CovarianceMonitorNode(Node):
    """AMCLの共分散をモニタリングして記録するノード"""

    def __init__(self):
        super().__init__('covariance_monitor')
        # パラメータの設定
        self.declare_parameter('rotation_count', 3)  # 回転回数
        self.declare_parameter('angular_speed', 0.4)  # rad/s
        self.declare_parameter('rotation_angle', 360.0)  # 度単位
        self.declare_parameter('wait_between_rotations', 2.0)  # 回転間の待機時間

        # QoSプロファイル設定
        cmd_vel_qos = QoSProfile(
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
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, amcl_qos)

        # パラメータ取得
        self.rotation_count = self.get_parameter('rotation_count').value
        self.angular_speed = self.get_parameter('angular_speed').value  # rad/s
        self.rotation_angle = math.radians(self.get_parameter('rotation_angle').value)
        self.wait_between_rotations = self.get_parameter('wait_between_rotations').value

        # データ収集用変数
        self.covariance_data = []
        self.current_rotation = 0
        self.rotation_start_time = 0
        self.is_rotating = False
        self.data_lock = threading.Lock()
        
        # CSVファイルパス設定
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filepath = f'/tmp/amcl_covariance_{timestamp}.csv'
        
        # CSVヘッダー書き込み
        with open(self.csv_filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'rotation_number', 'elapsed_seconds', 'covariance_35', 'position_x', 'position_y', 'orientation_z'])
        
        self.get_logger().info(
            f'共分散モニターを開始します:\n'
            f'  - 回転回数: {self.rotation_count}回\n'
            f'  - 角速度: {self.angular_speed} rad/s ({math.degrees(self.angular_speed):.1f}度/秒)\n'
            f'  - 回転角度: {math.degrees(self.rotation_angle):.1f}度\n'
            f'  - データ記録先: {self.csv_filepath}'
        )
        
        # 12秒後に実行を開始（AMCL起動とinitial_pose設定の後）
        self.timer = self.create_timer(12.0, self.start_monitoring)
        self.timer.cancel()  # 一度だけ実行するためにキャンセル
        
    def start_monitoring(self):
        """モニタリングを開始"""
        self.get_logger().info('共分散モニタリングを開始します')
        self.timer.cancel()  # 一度だけ実行するためキャンセル
        # 最初の回転を開始
        self.perform_next_rotation()
        
    def amcl_pose_callback(self, msg):
        """AMCLからのポーズ情報を受信して記録"""
        with self.data_lock:
            if not self.is_rotating:
                return
                
            # 現在の回転の経過時間
            elapsed = time.time() - self.rotation_start_time
            
            # 共分散データを記録
            yaw_covariance = msg.pose.covariance[35]
            position_x = msg.pose.pose.position.x
            position_y = msg.pose.pose.position.y
            orientation_z = msg.pose.pose.orientation.z
            
            # データをリストに追加
            self.covariance_data.append({
                'timestamp': time.time(),
                'rotation': self.current_rotation,
                'elapsed': elapsed,
                'covariance': yaw_covariance,
                'position_x': position_x,
                'position_y': position_y,
                'orientation_z': orientation_z
            })
            
            # CSVに書き込み
            with open(self.csv_filepath, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    time.time(),
                    self.current_rotation,
                    elapsed,
                    yaw_covariance,
                    position_x,
                    position_y,
                    orientation_z
                ])
            
            # 10データごとにログ出力
            if len(self.covariance_data) % 10 == 0:
                self.get_logger().debug(
                    f'回転{self.current_rotation}：共分散 = {yaw_covariance:.6f}, '
                    f'経過時間 = {elapsed:.2f}秒'
                )

    def perform_next_rotation(self):
        """次の回転を実行"""
        if self.current_rotation >= self.rotation_count:
            self.finish_monitoring()
            return
            
        self.current_rotation += 1
        self.get_logger().info(f'回転 {self.current_rotation}/{self.rotation_count} を開始します')
        
        # 回転実行
        self.is_rotating = True
        self.rotation_start_time = time.time()
        self.perform_rotation()
        
    def perform_rotation(self):
        """ロボットを回転させる"""
        try:
            self.stop_robot()
            time.sleep(0.5)
            
            twist_msg = Twist()
            twist_msg.angular.z = self.angular_speed
            rotation_time = abs(self.rotation_angle / self.angular_speed)
            
            start_time = time.time()
            end_time = start_time + rotation_time
            
            while time.time() < end_time:
                self.cmd_vel_pub.publish(twist_msg)
                time.sleep(0.05)
            
            self.stop_robot()
            self.get_logger().info(f'回転 {self.current_rotation} 完了')
            
            # 回転間の待機時間後に次の回転
            self.get_logger().info(f'{self.wait_between_rotations}秒待機します')
            time.sleep(self.wait_between_rotations)
            
            # 次の回転を実行
            self.perform_next_rotation()
            
        except Exception as e:
            self.get_logger().error(f'回転中にエラーが発生しました: {str(e)}')
            self.stop_robot()
        
    def stop_robot(self):
        """ロボットを停止させる"""
        stop_msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.1)
            
    def finish_monitoring(self):
        """モニタリングを終了し、結果を表示"""
        self.is_rotating = False
        
        # 各回転ごとの平均共分散を計算
        rotation_stats = {}
        for data in self.covariance_data:
            rotation = data['rotation']
            if rotation not in rotation_stats:
                rotation_stats[rotation] = {
                    'sum': 0,
                    'count': 0,
                    'min': float('inf'),
                    'max': float('-inf')
                }
            
            stats = rotation_stats[rotation]
            cov = data['covariance']
            stats['sum'] += cov
            stats['count'] += 1
            stats['min'] = min(stats['min'], cov)
            stats['max'] = max(stats['max'], cov)
        
        # 結果をログに出力
        self.get_logger().info('共分散モニタリング完了')
        self.get_logger().info(f'データ記録先: {self.csv_filepath}')
        self.get_logger().info('--- 回転ごとの共分散統計 ---')
        
        for rotation, stats in rotation_stats.items():
            avg = stats['sum'] / stats['count'] if stats['count'] > 0 else 0
            self.get_logger().info(
                f'回転 {rotation}: 平均={avg:.6f}, 最小={stats["min"]:.6f}, '
                f'最大={stats["max"]:.6f}, サンプル数={stats["count"]}'
            )
        
        degrees_uncertainty = math.degrees(math.sqrt(rotation_stats[self.rotation_count]['sum'] / rotation_stats[self.rotation_count]['count']))
        self.get_logger().info(f'最終回転後の平均方向不確かさ: 約 {degrees_uncertainty:.2f}度')


def main():
    rclpy.init()
    node = CovarianceMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()