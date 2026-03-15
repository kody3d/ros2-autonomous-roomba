#!/usr/bin/env python3
# initial_rotation.py - AMCLの自己位置推定精度向上のための初期回転運動を実行

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
import sys
import math
import threading

class InitialRotation(Node):
    def __init__(self):
        super().__init__('initial_rotation')
        
        # 【修正】回転パラメータを調整
        # 回転速度をやや低く、回転角度を360度に拡大、往復を無効化
        self.declare_parameter('angular_speed', 0.3)        # rad/s (約17度/秒)
        self.declare_parameter('rotation_angle', 360.0)     # 180度の大きな回転に変更
        self.declare_parameter('clockwise', True)           # 時計回り設定を追加
        self.declare_parameter('back_and_forth', False)     # 往復回転を無効化
        self.declare_parameter('retry_count', 1)            # 回転試行回数
        
        # 【追加】AMCLの収束判定パラメータ
        self.declare_parameter('monitor_convergence', True)        # 収束監視を有効にするか
        self.declare_parameter('convergence_threshold', 0.05)      # 共分散の閾値 [rad²]
        self.declare_parameter('convergence_timeout', 30.0)        # 収束監視のタイムアウト [秒]
        self.declare_parameter('min_convergence_duration', 1.0)    # この期間閾値以下なら収束と判断 [秒]
        
        # QoSプロファイル設定
        qos_profile = QoSProfile(
            depth=10,  # キューの深さ
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # cmd_velパブリッシャーの作成
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )
        
        # 【追加】AMCL位置推定結果のサブスクライバーを作成
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            qos_profile
        )
        
        # initial_pose パブリッシャーの作成
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile
        )
        
        # 【追加】収束判定用変数
        self.yaw_covariance = float('inf')  # 初期値は無限大
        self.convergence_time = None        # 収束開始時刻
        self.is_converged = False           # 収束フラグ
        self.convergence_lock = threading.Lock()  # スレッドセーフな処理のためのロック
        
        # パラメータの取得
        self.angular_speed = self.get_parameter('angular_speed').value  # rad/s
        self.rotation_angle = math.radians(self.get_parameter('rotation_angle').value)  # ラジアンに変換
        self.clockwise = self.get_parameter('clockwise').value
        self.back_and_forth = self.get_parameter('back_and_forth').value
        self.retry_count = self.get_parameter('retry_count').value
        
        # 【追加】収束判定パラメータを取得
        self.monitor_convergence = self.get_parameter('monitor_convergence').value
        self.convergence_threshold = self.get_parameter('convergence_threshold').value
        self.convergence_timeout = self.get_parameter('convergence_timeout').value
        self.min_convergence_duration = self.get_parameter('min_convergence_duration').value
        
        # 時計回りなら角速度を負に
        if self.clockwise:
            self.angular_speed = -self.angular_speed
        
        # 詳細なログ出力
        self.get_logger().info(
            f'【強化版】初期回転運動を実行します:\n'
            f'  - 角速度: {self.angular_speed} rad/s ({math.degrees(self.angular_speed):.1f}度/秒)\n'
            f'  - 回転角度: {math.degrees(self.rotation_angle):.1f}度\n'
            f'  - 回転方向: {"時計回り" if self.clockwise else "反時計回り"}\n'
            f'  - 往復回転: {"有効" if self.back_and_forth else "無効"}\n'
            f'  - 試行回数: {self.retry_count}回\n'
            f'  - 収束監視: {"有効" if self.monitor_convergence else "無効"}\n'
            f'  - 収束閾値: {self.convergence_threshold} rad² (約{math.degrees(math.sqrt(self.convergence_threshold)):.1f}度)'
        )
    # メッセージを受け取ったときに実行されるコールバック関数
    def amcl_pose_callback(self, msg):
        try:
            # yaw方向の共分散を取得 (共分散行列の35番目の要素)
            yaw_covariance = msg.pose.covariance[35]
            
            self.get_logger().info(f'共分散 = {yaw_covariance:.5f} rad²')
        except Exception as e:
            self.get_logger().error(f'コールバックでエラー: {str(e)}')

 

    def stop_robot(self):
        """ロボットを停止させる"""
        stop_msg = Twist()
        # すべての速度をゼロに
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        
        # 停止コマンドを複数回送信（確実に停止させるため）
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.1)
        
        self.get_logger().debug('ロボットを停止しました')

    def publish_initial_pose(self, pose_msg):
        """
        初期位置をパブリッシュして少し待つ
        
        Args:
            pose_msg: パブリッシュする初期位置のPoseWithCovarianceStampedメッセージ
        """
        # メッセージをパブリッシュ
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info('initial_poseをパブリッシュしました')
        
        # パブリッシュ後に少し待機（AMCLが処理する時間を確保）
        time.sleep(0.5)
        self.get_logger().info('initial_poseパブリッシュ後の待機完了')

def main():
    rclpy.init()
    
    try:
        node = InitialRotation()
        node.get_logger().info('=== 初期回転ノード起動 - AMCL位置推定の収束監視機能付き ===')
        
        # 初期位置メッセージを作成する例
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = node.get_clock().now().to_msg()
        # 位置と姿勢の設定...
        
        # 初期位置をパブリッシュして待機
        node.publish_initial_pose(initial_pose)
                # ここでノードをスピンさせる必要があります
        rclpy.spin(node)
                
        node.destroy_node()
        return 0
    
    except Exception as e:
        print(f'予期せぬエラーが発生しました: {str(e)}')
        return 1
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main())