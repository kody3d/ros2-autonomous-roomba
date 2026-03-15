#!/usr/bin/env python3
# initial_pose_publisher.py - AMCLの準備確認と初期位置設定の再試行機能

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState  # 修正: nav2_msgs -> lifecycle_msgs
from lifecycle_msgs.msg import State
import time
import sys

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('variance', 0.5)
        self.declare_parameter('max_attempts', 1)
        self.declare_parameter('wait_between_attempts', 2.0)
        
        # AMCLの状態を確認するためのクライアント
        self.amcl_state_client = self.create_client(GetState, '/amcl/get_state')
        
        # 初期位置パブリッシャー
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
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
            import math
            self.pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            self.pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 共分散行列（確信度）
        # 小さい値 = 確信度が高い
        variance = self.get_parameter('variance').value
        self.pose_msg.pose.covariance[0] = variance   # x
        self.pose_msg.pose.covariance[7] = variance   # y
        self.pose_msg.pose.covariance[35] = variance  # theta
        
        # 最大試行回数とタイムアウト
        self.max_attempts = self.get_parameter('max_attempts').value
        self.wait_between_attempts = self.get_parameter('wait_between_attempts').value
        
        self.get_logger().info(f'initial_pose_publisher を起動しました (x:{self.pose_msg.pose.pose.position.x}, '
                               f'y:{self.pose_msg.pose.pose.position.y})')

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
        
        # 初期位置を複数回送信（確実に適用されるように）
        attempt = 0
        while attempt < self.max_attempts:
            attempt += 1
            
            # 現在時刻でタイムスタンプを更新
            self.pose_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 初期位置を発行
            self.publisher.publish(self.pose_msg)
            self.get_logger().info(f'初期位置を発行しました (試行 {attempt}/{self.max_attempts})')
            
            # 間を置いて再送信
            time.sleep(self.wait_between_attempts)
        
        self.get_logger().info('初期位置設定完了')
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
