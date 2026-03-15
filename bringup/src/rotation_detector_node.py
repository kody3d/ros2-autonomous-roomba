#!/usr/bin/env python3
# rotation_detector_node.py - スキャンデータに基づいて回転方向を推定するノード

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
import time

class RotationDetector(Node):
    def __init__(self):
        super().__init__('rotation_detector')
        
        # パラメータ宣言
        self.declare_parameter('max_detection_time', 10.0)  # 検出最大時間(秒)
        self.declare_parameter('min_match_score', 0.65)     # マッチング最小スコア
        self.declare_parameter('scan_topic', '/scan')       # スキャントピック
        self.declare_parameter('initial_pose_topic', '/initialpose')  # 初期位置トピック
        
        # パラメータ取得
        self.max_detection_time = self.get_parameter('max_detection_time').value
        self.min_match_score = self.get_parameter('min_match_score').value
        scan_topic = self.get_parameter('scan_topic').value
        initial_pose_topic = self.get_parameter('initial_pose_topic').value
        
        # サブスクライバーとパブリッシャーの設定
        self.scan_sub = self.create_subscription(
            LaserScan, 
            scan_topic, 
            self.scan_callback, 
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            initial_pose_topic,
            10
        )
        
        # スキャンデータ蓄積用
        self.scans = []
        self.max_scans = 5  # 蓄積する最大スキャン数
        self.is_detecting = True
        self.start_time = time.time()
        
        # TF関連の設定
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('回転方向検出ノードが起動しました')
        
    def scan_callback(self, msg):
        """スキャンデータが届いたときのコールバック"""
        # 検出フェーズが終了したらスキップ
        if not self.is_detecting:
            return
            
        # タイムアウトチェック
        if time.time() - self.start_time > self.max_detection_time:
            self.get_logger().warn(f'検出タイムアウト: {self.max_detection_time}秒が経過しました')
            self.is_detecting = False
            if len(self.scans) > 0:
                self.process_collected_scans()
            return
            
        # スキャン前処理
        processed_scan = self.preprocess_scan(msg)
        
        # スキャン蓄積
        self.scans.append(processed_scan)
        self.get_logger().debug(f'スキャンを蓄積しました ({len(self.scans)}/{self.max_scans})')
        
        # 十分なスキャンが蓄積されたら処理を開始
        if len(self.scans) >= self.max_scans:
            self.is_detecting = False
            self.process_collected_scans()
    
    def preprocess_scan(self, scan_msg):
        """スキャンデータの前処理"""
        ranges = np.array(scan_msg.ranges)
        
        # 無効な値を処理
        invalid_indices = np.isnan(ranges) | np.isinf(ranges)
        ranges[invalid_indices] = scan_msg.range_max
        
        # 距離でフィルタリング
        ranges[ranges < scan_msg.range_min] = scan_msg.range_max
        ranges[ranges > scan_msg.range_max] = scan_msg.range_max
        
        # 角度情報も保存
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        angles = np.array([angle_min + i * angle_increment for i in range(len(ranges))])
        
        return {
            'ranges': ranges,
            'angles': angles,
            'time': scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9,
            'frame_id': scan_msg.header.frame_id
        }
    
    def process_collected_scans(self):
        """蓄積したスキャンを処理して回転方向を推定"""
        self.get_logger().info(f'蓄積した{len(self.scans)}個のスキャンを処理します')
        
        # スキャンが少なすぎる場合は中止
        if len(self.scans) < 2:
            self.get_logger().warn('スキャン数が不足しています。回転推定を中止します')
            return
            
        # 基準スキャン（最初のもの）
        reference_scan = self.scans[0]
        
        # 最適な回転角度を探索（8方向）
        angles_to_try = [0, np.pi/4, np.pi/2, 3*np.pi/4, np.pi, -3*np.pi/4, -np.pi/2, -np.pi/4]
        best_score = 0.0
        best_angle = 0.0
        
        for angle in angles_to_try:
            score = self.calculate_rotation_match(reference_scan, angle)
            self.get_logger().debug(f'回転角 {angle:.2f} rad のスコア: {score:.4f}')
            
            if score > best_score:
                best_score = score
                best_angle = angle
        
        # 最適な角度が見つかったかどうか確認
        if best_score > self.min_match_score:
            self.get_logger().info(f'最適な回転角を検出: {best_angle:.4f} rad (スコア: {best_score:.4f})')
            self.publish_initial_pose(best_angle)
        else:
            self.get_logger().warn(f'確信度の高い回転角が見つかりませんでした (最高スコア: {best_score:.4f})')
    
    def calculate_rotation_match(self, reference_scan, rotation_angle):
        """特定の回転角でのマッチングスコアを計算"""
        # 参照スキャンと他のスキャンを比較
        total_score = 0.0
        
        for scan in self.scans[1:]:
            # 指定角度だけ回転させたスキャンを生成
            rotated_scan = self.rotate_scan(scan, rotation_angle)
            
            # スキャン間の一致度を計算
            match_score = self.compare_scans(reference_scan, rotated_scan)
            total_score += match_score
        
        # 平均スコアを計算
        return total_score / (len(self.scans) - 1) if len(self.scans) > 1 else 0.0
    
    def rotate_scan(self, scan, angle):
        """スキャンデータを指定角度だけ回転させる"""
        # スキャンの角度を回転
        rotated_angles = scan['angles'] + angle
        
        # 角度を-πからπの範囲に正規化
        rotated_angles = np.mod(rotated_angles + np.pi, 2 * np.pi) - np.pi
        
        # 元のスキャンのコピーを作成
        rotated_scan = scan.copy()
        rotated_scan['angles'] = rotated_angles
        
        return rotated_scan
    
    def compare_scans(self, scan1, scan2):
        """2つのスキャンの類似度を計算"""
        # 角度をそろえる必要がある
        ranges1 = scan1['ranges']
        angles1 = scan1['angles']
        ranges2 = scan2['ranges']
        angles2 = scan2['angles']
        
        # 共通の角度グリッドを作成
        common_angles = np.linspace(-np.pi, np.pi, 360)
        
        # 各スキャンのレンジを共通角度にリサンプリング
        interp_ranges1 = np.interp(common_angles, angles1, ranges1, period=2*np.pi)
        interp_ranges2 = np.interp(common_angles, angles2, ranges2, period=2*np.pi)
        
        # 障害物があるポイントのみを抽出（max_rangeでない点）
        valid_indices = (interp_ranges1 < 12.0) & (interp_ranges2 < 12.0)
        
        if np.sum(valid_indices) < 10:  # 有効点が少なすぎる場合
            return 0.0
            
        r1 = interp_ranges1[valid_indices]
        r2 = interp_ranges2[valid_indices]
        
        # 距離の差異を計算
        diff = np.abs(r1 - r2)
        similarity = np.exp(-diff.mean())  # 差異が小さいほど類似度は高い
        
        return similarity
    
    def publish_initial_pose(self, yaw):
        """検出した回転角を使用して初期位置を発行"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # 位置は現在のオドメトリから取得するか、既定値を使用
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        
        # 四元数に変換
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 共分散行列（位置には高い不確かさ、角度には低い不確かさ）
        pose_msg.pose.covariance = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.25]
        
        self.pose_pub.publish(pose_msg)
        self.get_logger().info('検出した回転に基づく初期位置を発行しました')

def main(args=None):
    rclpy.init(args=args)
    node = RotationDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()