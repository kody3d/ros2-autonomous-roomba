#!/usr/bin/env python3
# initial_pose_publisher.py - AMCLの準備確認と初期位置設定の再試行機能

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState  # 修正: nav2_msgs -> lifecycle_msgs
from lifecycle_msgs.msg import State
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
import sys
import math  # mathモジュールは最初からインポート

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        
        # 【変更】許容誤差の分散値設定
        # 元の設定: variance = 0.5
        #  - 位置の許容誤差: 0.71メートル（√0.5）
        #  - 方向の許容誤差: 40.6度（√0.5ラジアン）
        #
        # 新しい設定:
        #  - 位置の許容誤差: 1.0メートル（√1.0）
        #  - 方向の許容誤差: 135度（√5.55ラジアン）
        
        # 元の値を残しつつ、コメントアウト
        # self.declare_parameter('variance', 0.5)
        
        # 新しい値で設定
        position_variance = 1.0  # 1.0m²（標準偏差1.0m）
        rotation_variance = 5.55  # 135度相当（約2.35ラジアン）² 
        self.declare_parameter('variance', position_variance)  # 位置誤差用の値を使用
        
        # 【修正】試行回数を増やして確実に届くようにする
        self.declare_parameter('max_attempts', 5)  # 1回→5回に増加
        self.declare_parameter('wait_between_attempts', 0.5)  # 間隔を短く
        
        # AMCLの状態を確認するためのクライアント
        self.amcl_state_client = self.create_client(GetState, '/amcl/get_state')
        
        # QoSプロファイル設定部分
        # 【修正】QoSプロファイルを明示的に設定
        qos_profile = QoSProfile(
            depth=10,  # キューの深さ
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 初期位置パブリッシャー
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile  # QoSプロファイルを指定
        )
        
        # 初期位置メッセージの作成
        self.pose_msg = PoseWithCovarianceStamped()
        self.pose_msg.header.frame_id = 'map'
        
        # パラメータから初期位置を読み込み
        self.pose_msg.pose.pose.position.x = self.get_parameter('x').value
        self.pose_msg.pose.pose.position.y = self.get_parameter('y').value
        
        # 四元数に変換 (yawのみ)
        yaw = self.get_parameter('yaw').value
        if yaw == 0.0:
            # 簡易的な設定
            self.pose_msg.pose.pose.orientation.w = 1.0
        else:
            # yawから四元数を計算
            self.pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            self.pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 【変更】共分散行列（確信度）設定
        # 小さい値 = 確信度が高い、大きい値 = 不確かさが大きい
        
        # 元々のコード（単一の分散値で全方向を設定）
        # variance = self.get_parameter('variance').value  # 元: 0.5
        # self.pose_msg.pose.covariance[0] = variance   # x
        # self.pose_msg.pose.covariance[7] = variance   # y
        # self.pose_msg.pose.covariance[35] = variance  # theta
        
        # 更新されたコード（位置と回転で異なる値を使用）
        # 位置の分散値は元々の設定値を使用
        position_variance = self.get_parameter('variance').value  # 新: 1.0
        # 回転の分散値は135度に相当する値（約5.55 rad²）
        rotation_variance = 5.55  # math.radians(135) ** 2
        
        # 全ての共分散値を0に初期化
        covariance = [0.0] * 36  # 6x6 = 36要素
        
        # 位置の不確かさ設定（x, y方向は1mの誤差、z方向はほぼ固定）
        covariance[0] = position_variance   # x（1.0m²）
        covariance[7] = position_variance   # y（1.0m²）
        covariance[14] = 0.01               # z（ほぼ固定）
        
        # 回転の不確かさ設定（roll, pitchはほぼ固定、yawは135度の許容）
        covariance[21] = 0.01               # roll（ほぼ固定）
        covariance[28] = 0.01               # pitch（ほぼ固定）
        covariance[35] = rotation_variance  # yaw（約135度の許容）
        
        # 共分散行列を設定
        self.pose_msg.pose.covariance = covariance
        
        # 最大試行回数とタイムアウト
        self.max_attempts = self.get_parameter('max_attempts').value
        self.wait_between_attempts = self.get_parameter('wait_between_attempts').value
        
        # より詳細なログ出力
        self.get_logger().info(
            f'initial_pose_publisher を起動しました (x:{self.pose_msg.pose.pose.position.x}, y:{self.pose_msg.pose.pose.position.y})\n'
            f'【変更】許容誤差設定:\n'
            f'  - 位置誤差: {math.sqrt(position_variance):.2f}m (元: 0.71m)\n'
            f'  - 回転許容: {math.degrees(math.sqrt(rotation_variance)):.1f}度 (元: 40.6度)'
        )

    def wait_for_amcl_active(self, timeout=30.0):
        """AMCLノードがアクティブになるまで待機"""
        start_time = time.time()
        
        # サービスの準備を待つ
        while not self.amcl_state_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout:
                self.get_logger().error('AMCLステート確認サービスが利用できません')
                return False
            
            self.get_logger().info('AMCLサービスを待機中...')
        
        # AMCLの状態を確認
        req = GetState.Request()
        
        while time.time() - start_time < timeout:
            future = self.amcl_state_client.call_async(req)
            
            # 応答を待つ
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                state = future.result().current_state.id
                
                if state == State.PRIMARY_STATE_ACTIVE:
                    self.get_logger().info('AMCLがアクティブ状態になりました')
                    return True
                
                self.get_logger().info(f'AMCL現在の状態: {state} (待機中)')
            else:
                self.get_logger().warn('AMCLの状態取得に失敗しました')
            
            time.sleep(1.0)
        
        self.get_logger().error('AMCLがアクティブになるのを待つタイムアウト')
        return False

    def publish_initial_pose(self):
        """初期位置を設定し、確実に適用されるまで再試行"""
        
        # AMCLが準備完了するまで待機
        if not self.wait_for_amcl_active():
            self.get_logger().warn('AMCLの準備ができないため、初期位置設定をスキップします')
            return False
        
        # 【修正】より明確なログを出力
        self.get_logger().info(f'初期位置の設定を開始します（{self.max_attempts}回試行）')
        
        # 初期位置を複数回送信（確実に適用されるように）
        attempt = 0
        while attempt < self.max_attempts:
            attempt += 1
            
            # 現在時刻でタイムスタンプを更新
            self.pose_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 初期位置を発行
            self.publisher.publish(self.pose_msg)
            self.get_logger().info(f'初期位置を発行しました (試行 {attempt}/{self.max_attempts}) - topic: /initialpose')
            
            # 間を置いて再送信
            time.sleep(self.wait_between_attempts)
        
        # 【修正】発行確認のためにさらに詳細なログを出力
        self.get_logger().info(
            f'初期位置設定完了 - 位置: (x:{self.pose_msg.pose.pose.position.x}, y:{self.pose_msg.pose.pose.position.y}), '
            f'向き: {self.pose_msg.pose.pose.orientation.w}'
        )
        
        # 【修正】確実に処理が完了するまで少し待機
        time.sleep(1.0)
        
        return True

def main():
    rclpy.init()
    
    try:
        node = InitialPosePublisher()
        success = node.publish_initial_pose()
        node.destroy_node()
        
        # 成功/失敗に基づいて終了コード設定
        if success:
            return 0
        else:
            return 1
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main())
