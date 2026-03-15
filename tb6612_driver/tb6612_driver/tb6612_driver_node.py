#!/usr/bin/env python3
"""
TB6612 ROS2 Driver Node

TB6612モータードライバによる2輪差動駆動ロボット制御のROS2ドライバ
Raspberry Pi Pico WとUART通信でコマンド送信とオドメトリ受信を行う
"""

import rclpy
from rclpy.node import Node
import serial
import json
import threading
import time
import math
import zlib  # CRC32計算用（ファイル上部に追加）
from typing import Optional, Dict, Any
import binascii

# ROS2 Messages
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# TF2 - ROS2 Humble対応
from tf2_ros import TransformBroadcaster

POSE_COVARIANCE = [
    1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5,
    1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5,
    0.0,  0.0,  1e-5, 0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1e-5, 0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,  1e-5, 0.0,
    1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5
]

TWIST_COVARIANCE = [
    1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5,
    1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5,
    0.0,  0.0,  1e-5, 0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1e-5, 0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,  1e-5, 0.0,
    1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5
]


def euler_to_quaternion(roll, pitch, yaw):
    """
    オイラー角からクォータニオンに変換（ROS2 Humble対応）
    
    Args:
        roll, pitch, yaw: ラジアン単位の回転角
    
    Returns:
        tuple: (x, y, z, w) クォータニオン
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)


class TB6612Driver(Node):
    """TB6612 Motor Driver ROS2 Node"""
    
    def __init__(self):
        super().__init__('tb6612_driver')
        
        # パラメータ宣言
        self._declare_parameters()
        
        # 内部状態
        self._serial_port: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._last_odom_data = {}
        self._last_cmd_vel_time = self.get_clock().now()
        self._connection_lost = False
        self._consecutive_errors = 0
        self._max_consecutive_errors = 5
        
        # オドメトリ計算用
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_time = self.get_clock().now()
        
        # Publishers
        self._setup_publishers()
        
        # Subscribers  
        self._setup_subscribers()
        
        # Services
        self._setup_services()
        
        # TF Broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)
        
        # ★Static TF設定を追加
        self._setup_static_transforms()
        
        # シリアル接続開始
        self._connect_serial()
        
        # タイマー設定
        self._setup_timers()
        
        self.get_logger().info('[TB6612] Driver initialized')
    
    def _declare_parameters(self):
        """パラメータを宣言 - CREATEドライバー手法を参考にした改良版"""
        # シリアル通信設定
        self.declare_parameter('serial_port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout', 0.1)
        
        # フレーム設定（CREATEドライバーと同様）
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)  # 条件付きTF配信制御
        
        # 制御設定
        self.declare_parameter('cmd_vel_timeout', 0.5)  # 秒
        self.declare_parameter('max_linear_vel', 0.5)   # m/s
        self.declare_parameter('max_angular_vel', 3.14) # rad/s
        
        # 診断設定
        self.declare_parameter('diagnostic_period', 1.0)  # 秒
        
        # ロボット物理パラメータ（CREATEドライバーと同様の考え方）
        self.declare_parameter('wheel_base', 0.16)  # m (車輪間距離)
        self.declare_parameter('wheel_radius', 0.033)  # m (車輪半径)
        
        # ★新規追加：フレーム位置設定
        self.declare_parameter('laser_offset_x', 0.1)
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('laser_offset_z', 0.05)
        self.declare_parameter('imu_offset_x', 0.0)
        self.declare_parameter('imu_offset_y', 0.0)
        self.declare_parameter('imu_offset_z', 0.02)
    
    def _setup_publishers(self):
        """パブリッシャーを設定"""
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._raw_data_pub = self.create_publisher(String, 'raw_data', 10)
    
    def _setup_subscribers(self):
        """サブスクライバーを設定"""
        self._cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10)
    
    def _setup_services(self):
        """サービスを設定"""
        self._stop_service = self.create_service(
            Trigger, 'stop_robot', self._stop_service_callback)
        self._reset_odom_service = self.create_service(
            Trigger, 'reset_odometry', self._reset_odom_service_callback)


    def _publish_diagnostics(self):
        """診断情報を配信"""
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # シリアル接続状態
            serial_status = DiagnosticStatus()
            serial_status.name = "TB6612 Serial Connection"
            serial_status.hardware_id = "tb6612_driver"
            
            if self._connection_lost or not self._serial_port or not self._serial_port.is_open:
                serial_status.level = DiagnosticStatus.ERROR
                serial_status.message = "Serial connection lost"
            elif self._consecutive_errors > 0:
                serial_status.level = DiagnosticStatus.WARN
                serial_status.message = f"Serial errors detected: {self._consecutive_errors}"
            else:
                serial_status.level = DiagnosticStatus.OK
                serial_status.message = "Serial connection healthy"
            
            # パラメータ情報
            serial_status.values.append(KeyValue(
                key="port", value=self.get_parameter('serial_port').value))
            serial_status.values.append(KeyValue(
                key="baud_rate", value=str(self.get_parameter('baud_rate').value)))
            serial_status.values.append(KeyValue(
                key="consecutive_errors", value=str(self._consecutive_errors)))
            
            diag_array.status.append(serial_status)
            
            # オドメトリ状態
            odom_status = DiagnosticStatus()
            odom_status.name = "TB6612 Odometry"
            odom_status.hardware_id = "tb6612_driver"
            
            if self._last_odom_data:
                odom_status.level = DiagnosticStatus.OK
                odom_status.message = "Odometry data available"
                
                # 最新オドメトリ値
                odom_status.values.append(KeyValue(
                    key="x", value=str(self._last_odom_data.get('x', 0))))
                odom_status.values.append(KeyValue(
                    key="y", value=str(self._last_odom_data.get('y', 0))))
                odom_status.values.append(KeyValue(
                    key="angle", value=str(self._last_odom_data.get('a', 0))))
                odom_status.values.append(KeyValue(
                    key="linear_vel", value=str(self._last_odom_data.get('lv', 0))))
                odom_status.values.append(KeyValue(
                    key="angular_vel", value=str(self._last_odom_data.get('av', 0))))
            else:
                odom_status.level = DiagnosticStatus.WARN
                odom_status.message = "No odometry data received"
            
            diag_array.status.append(odom_status)
            
            self._diagnostic_pub.publish(diag_array)
            
        except Exception as e:
            self.get_logger().warn(f'[TB6612] Error publishing diagnostics: {e}')

    def _setup_timers(self):
        """タイマーを設定"""
        # 診断タイマー
        diagnostic_period = self.get_parameter('diagnostic_period').value
        self._diagnostic_timer = self.create_timer(
            diagnostic_period, self._publish_diagnostics)
        
        # 安全停止チェックタイマー
        self._safety_timer = self.create_timer(0.1, self._safety_check)
    
    def _connect_serial(self):
        """シリアル接続を確立"""
        try:
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('baud_rate').value
            timeout = self.get_parameter('serial_timeout').value
            
            self._serial_port = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=timeout,
                write_timeout=timeout,
                parity=serial.PARITY_NONE,  # パリティなし
                stopbits=serial.STOPBITS_ONE,  # ストップビット1
                bytesize=serial.EIGHTBITS  # データビット8
            )
            
            # シリアル受信スレッド開始
            self._serial_thread = threading.Thread(
                target=self._serial_read_loop, daemon=True)
            self._serial_thread.start()
            
            self.get_logger().info(f'[TB6612] Serial connected: {port} @ {baud}')
            self._connection_lost = False
            self._consecutive_errors = 0
            
        except Exception as e:
            self.get_logger().error(f'[TB6612] Failed to connect serial: {e}')
            self._connection_lost = True
    
    def _serial_read_loop(self):
        """シリアル受信ループ（別スレッド）"""
        while rclpy.ok() and self._serial_port and self._serial_port.is_open:
            try:
                if self._serial_port.in_waiting > 0:
                    line = self._serial_port.readline().decode('utf-8').strip()
                    if line:
                        self._process_received_data(line)
                        self._consecutive_errors = 0
                        
            except serial.SerialException as e:
                self.get_logger().warn(f'[TB6612] Serial read error: {e}')
                self._consecutive_errors += 1
                if self._consecutive_errors >= self._max_consecutive_errors:
                    self.get_logger().error('[TB6612] Serial connection lost')
                    self._connection_lost = True
                    break
                
            except Exception as e:
                self.get_logger().warn(f'[TB6612] Unexpected error in serial loop: {e}')


    def _calculate_crc32_checksum(self, data: str) -> str:
        """CRC32チェックサム計算"""
        crc = zlib.crc32(data.encode('utf-8')) & 0xFFFFFFFF
        return f"{crc:08x}"  # 小文字で返す
    
    def _process_received_data(self, data: str):
        """受信データを処理（CRC32チェックサム検証付き）"""
        try:

            # ★受信データの詳細表示
            # self.get_logger().info(f'[RAW_RECV] 受信データ: "{data}"')
        
            # 生データをパブリッシュ
            raw_msg = String()
            raw_msg.data = data
            self._raw_data_pub.publish(raw_msg)
            
            # ★CRC32チェックサム検証処理
            json_str = data.strip()
            checksum_received = None
            
            # "#"でチェックサムが付いているかチェック
            if '#' in json_str:
                parts = json_str.split('#')
                if len(parts) == 2:
                    json_str = parts[0]  # JSONデータ部分
                    checksum_received = parts[1].lower()  # チェックサム部分（小文字化）
                    
                    self.get_logger().debug(f'[CRC32_RECV] JSONデータ: "{json_str}"')
                    self.get_logger().debug(f'[CRC32_RECV] 受信チェックサム: {checksum_received}')
                    
                    # チェックサム計算
                    checksum_calculated = self._calculate_crc32_checksum(json_str)
                    self.get_logger().debug(f'[CRC32_CALC] 計算チェックサム: {checksum_calculated}')
                    
                    # チェックサム検証
                    if checksum_received != checksum_calculated:
                        self.get_logger().warn(
                            f'[CRC32_ERROR] チェックサムエラー - データを破棄 '
                            f'期待値: {checksum_calculated}, 受信値: {checksum_received}, '
                            f'データ: "{json_str}"')
                        return  # チェックサムエラーの場合は処理せずに破棄
                    
                    self.get_logger().debug('[CRC32_OK] チェックサム検証成功')
                else:
                    self.get_logger().warn(f'[CRC32_FORMAT] 不正なチェックサム形式 - データを破棄: "{data}"')
                    return
            else:
                self.get_logger().debug('[CRC32_NONE] チェックサムなしのデータ')
        
            # 末尾カンマ問題も対応
            if json_str.endswith(',}'):
                json_str = json_str[:-2] + '}'
                self.get_logger().debug(f'[JSON_CLEAN] カンマ除去: "{json_str}"')
        
            # JSON解析
            json_data = json.loads(json_str)
            self.get_logger().debug(f'[JSON_SUCCESS] JSON解析成功: {json_data}')
            
                    # ★ACK応答は無視
            if 'ack' in json_data:
                self.get_logger().debug(f'[ACK] 応答メッセージを無視: {json_data}')
                return
            
            # オドメトリデータの処理
            self._process_odometry_data(json_data)
            
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'[JSON_ERROR] JSON解析失敗 - データを破棄: "{data}" - エラー: {e}')
            
        except Exception as e:
            self.get_logger().warn(f'[PROCESS_ERROR] データ処理エラー - データを破棄: "{data}" - エラー: {e}')
    
    def _process_odometry_data(self, data: Dict[str, Any]):
        """オドメトリデータを処理してROS2メッセージとして配信"""
        try:
            current_time = self.get_clock().now()
            
            if not all(k in data for k in ('x', 'y', 'a')):
                self.get_logger().warn(f'[ODOM] 欠損データ: {data}')

            # データ取得
            x = float(data.get('x', 0.0))
            y = float(data.get('y', 0.0))
            angle = float(data.get('a', 0.0))
            linear_vel = float(data.get('lv', 0.0))
            angular_vel = float(data.get('av', 0.0))
            
            # ★オドメトリ情報をlogger.info表示
            if linear_vel!=0.0 or angular_vel!=0.0:
                self.get_logger().info(f'[ODOM] 位置: ({x:.3f}, {y:.3f}) 角度: {angle:.3f}rad({math.degrees(angle):.1f}°) '
                    f'速度: L={linear_vel:.3f}m/s A={angular_vel:.3f}rad/s')
    
            
            # Odometryメッセージ作成（既存のコード）
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = self.get_parameter('odom_frame').value
            odom_msg.child_frame_id = self.get_parameter('base_frame').value
            
            # 位置設定
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            
            # 姿勢設定（クォータニオン）- ROS2 Humble対応
            quaternion = euler_to_quaternion(0, 0, angle)
            odom_msg.pose.pose.orientation.x = quaternion[0]
            odom_msg.pose.pose.orientation.y = quaternion[1]
            odom_msg.pose.pose.orientation.z = quaternion[2]
            odom_msg.pose.pose.orientation.w = quaternion[3]
            
            # 速度設定
            odom_msg.twist.twist.linear.x = linear_vel
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = angular_vel
            
            # 共分散設定（簡単な値）
            pose_covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            
            twist_covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            
            odom_msg.pose.covariance = pose_covariance
            odom_msg.twist.covariance = twist_covariance
            # 共分散設定（CREATEドライバーと同様の値を使用）
# 0.1の場合: AMCLがオドメトリをあまり信頼せず、Lidarを重視
# 1e-5の場合: AMCLがオドメトリを高く信頼し、素早い追従
            # odom_msg.pose.covariance = POSE_COVARIANCE
            # odom_msg.twist.covariance = TWIST_COVARIANCE
        
            # オドメトリ配信
            self._odom_pub.publish(odom_msg)
            
            # TF配信
            if self.get_parameter('publish_tf').value:
                self._publish_tf(odom_msg)
            
            # 内部状態更新
            self._last_odom_data = data
            
            # デバッグ情報（既存）
            self.get_logger().debug(
                f'[TB6612] Odom - x: {x:.3f}, y: {y:.3f}, theta: {angle:.3f}')
            
        except Exception as e:
            self.get_logger().warn(f'[TB6612] Error processing odometry: {e}')
    
    def _publish_tf(self, odom_msg: Odometry):
        """TF変換を配信 - CREATEドライバー手法を参考にした改良版"""
        try:
            # CREATEドライバーと同様の条件付きTF配信
            if not self.get_parameter('publish_tf').value:
                return
                
            # TFメッセージを作成
            tf_odom = TransformStamped()
            
            # ヘッダー設定（CREATEドライバーと同じパターン）
            tf_odom.header.stamp = odom_msg.header.stamp  # オドメトリと同じタイムスタンプ
            tf_odom.header.frame_id = self.get_parameter('odom_frame').value
            tf_odom.child_frame_id = self.get_parameter('base_frame').value
            
            # 変換データをコピー
            tf_odom.transform.translation.x = odom_msg.pose.pose.position.x
            tf_odom.transform.translation.y = odom_msg.pose.pose.position.y
            tf_odom.transform.translation.z = odom_msg.pose.pose.position.z
            tf_odom.transform.rotation = odom_msg.pose.pose.orientation
            
            # TF配信
            self._tf_broadcaster.sendTransform(tf_odom)
            
            self.get_logger().debug(
                f'[TB6612] TF published: {tf_odom.header.frame_id} → {tf_odom.child_frame_id}')

        except Exception as e:
            self.get_logger().warn(f'[TB6612] Error publishing TF: {e}')
    


    def _setup_static_transforms(self):
        """静的TF変換を設定 - CREATEドライバー手法を参考にした改良版"""
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        from geometry_msgs.msg import TransformStamped
        
        # Static TF Broadcasterを初期化（重複回避）
        if not hasattr(self, '_static_tf_broadcaster'):
            self._static_tf_broadcaster = StaticTransformBroadcaster(self)
            self.get_logger().info('[TB6612] Static TF Broadcaster initialized')
        
        # 現在時刻を取得
        current_time = self.get_clock().now().to_msg()
        
        # 静的変換リストを作成
        static_transforms = []
        
        # ★1. base_footprint → base_link変換
        tf_base_link = TransformStamped()
        tf_base_link.header.stamp = current_time
        tf_base_link.header.frame_id = 'base_footprint'
        tf_base_link.child_frame_id = 'base_link'
        
        # base_linkの位置設定（CREATEと同様の考え方）
        tf_base_link.transform.translation.x = 0.0
        tf_base_link.transform.translation.y = 0.0
        tf_base_link.transform.translation.z = 0.1  # 10cm上方
        
        # 回転なし（単位クォータニオン）
        tf_base_link.transform.rotation.x = 0.0
        tf_base_link.transform.rotation.y = 0.0
        tf_base_link.transform.rotation.z = 0.0
        tf_base_link.transform.rotation.w = 1.0
        
        static_transforms.append(tf_base_link)
        
        # ★2. base_link → laser_frame変換
        tf_laser = TransformStamped()
        tf_laser.header.stamp = current_time
        tf_laser.header.frame_id = 'base_link'
        tf_laser.child_frame_id = 'laser_frame'
        
        # LiDARの取り付け位置（パラメータから取得）
        tf_laser.transform.translation.x = self.get_parameter('laser_offset_x').value
        tf_laser.transform.translation.y = self.get_parameter('laser_offset_y').value
        tf_laser.transform.translation.z = self.get_parameter('laser_offset_z').value
        
        # 回転なし
        tf_laser.transform.rotation.x = 0.0
        tf_laser.transform.rotation.y = 0.0
        tf_laser.transform.rotation.z = 0.0
        tf_laser.transform.rotation.w = 1.0
        
        static_transforms.append(tf_laser)
        
        # ★3. base_link → imu_link変換（IMU使用時）
        tf_imu = TransformStamped()
        tf_imu.header.stamp = current_time
        tf_imu.header.frame_id = 'base_link'
        tf_imu.child_frame_id = 'imu_link'
        
        # IMUの取り付け位置（パラメータから取得）
        tf_imu.transform.translation.x = self.get_parameter('imu_offset_x').value
        tf_imu.transform.translation.y = self.get_parameter('imu_offset_y').value
        tf_imu.transform.translation.z = self.get_parameter('imu_offset_z').value
        
        # 回転なし
        tf_imu.transform.rotation.x = 0.0
        tf_imu.transform.rotation.y = 0.0
        tf_imu.transform.rotation.z = 0.0
        tf_imu.transform.rotation.w = 1.0
        
        static_transforms.append(tf_imu)
        
        # ★一括送信（CREATEドライバーと同様）
        self._static_tf_broadcaster.sendTransform(static_transforms)
        
        self.get_logger().info('[TB6612] Static TF published: base_footprint → base_link → {laser_frame, imu_link}')


    
    def _calculate_crc16(self, data: str) -> str:
        """CRC16-CCITT-FALSE (0x1021, 初期値0xFFFF) を計算し4桁16進文字列で返す"""
        crc = 0xFFFF
        for c in data.encode('utf-8'):
            crc ^= c << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return f"{crc:04x}"


    def _cmd_vel_callback(self, msg: Twist):
        """cmd_velコールバック（文字列SUM32版）"""
        try:
            # 速度制限
            max_linear = self.get_parameter('max_linear_vel').value
            max_angular = self.get_parameter('max_angular_vel').value

            linear = max(-max_linear, min(max_linear, msg.linear.x))
            angular = max(-max_angular, min(max_angular, msg.angular.z))
            angular *= 2.0
            

            # JSONコマンド作成
            cmd = {
                "l": round(linear, 3),
                "a": round(angular, 3),
            }

            json_str = json.dumps(cmd, separators=(',', ':'))

            # CRC16計算
            crc16 = self._calculate_crc16(json_str)

            # 送信文字列生成
            send_str = f"{json_str}#{crc16}\n"

            # debug確認
            # self.get_logger().info(send_str)
        

            # シリアル送信
            self._send_command(send_str)
            
            # 最終コマンド時刻更新
            self._last_cmd_vel_time = self.get_clock().now()
            

        except Exception as e:
            self.get_logger().warn(f'[TB6612] Error in cmd_vel callback: {e}')
    
    def _send_command(self, cmd: str):
        """コマンドをシリアル送信（文字列版）"""
        if not self._serial_port or not self._serial_port.is_open:
            self.get_logger().warn('[TB6612] Serial port not available')
            return False

        try:
            with self._serial_lock:
                # ここでcmdはすでにstr（改行付き）で渡される
                bytes_written = self._serial_port.write(cmd.encode('utf-8'))
                self._serial_port.flush()
                self.get_logger().debug(f'[TB6612] Sent {bytes_written} bytes: {cmd.strip()}')
                return True

        except Exception as e:
            self.get_logger().warn(f'[TB6612] Failed to send command: {e}')
            return False
    
    def _stop_service_callback(self, request, response):
        """停止サービスのコールバック"""
        try:
            cmd = {"cmd": "stop"}
            if self._send_command(cmd):
                response.success = True
                response.message = "Robot stopped successfully"
                self.get_logger().info('[TB6612] Robot stopped via service')
            else:
                response.success = False
                response.message = "Failed to send stop command"
                
        except Exception as e:
            response.success = False
            response.message = f"Stop service error: {e}"
            self.get_logger().error(f'[TB6612] Stop service error: {e}')
        
        return response
    
    def _reset_odom_service_callback(self, request, response):
        """オドメトリリセットサービスのコールバック"""
        try:
            cmd = {"cmd": "reset_odom"}
            if self._send_command(cmd):
                # 内部状態もリセット
                self._x = 0.0
                self._y = 0.0
                self._theta = 0.0
                self._last_odom_data = {}
                
                response.success = True
                response.message = "Odometry reset successfully"
                self.get_logger().info('[TB6612] Odometry reset via service')
            else:
                response.success = False
                response.message = "Failed to send reset command"
                
        except Exception as e:
            response.success = False
            response.message = f"Reset odometry service error: {e}"
            self.get_logger().error(f'[TB6612] Reset odometry service error: {e}')
        
        return response
    
    def _safety_check(self):
        """安全チェック（cmd_velタイムアウト、接続切断時の緊急停止）"""
        try:
            current_time = self.get_clock().now()
            timeout = self.get_parameter('cmd_vel_timeout').value
            
            # # cmd_velタイムアウトチェック
            # time_since_last_cmd = (current_time - self._last_cmd_vel_time).nanoseconds / 1e9
            
            # if time_since_last_cmd > timeout:
            #     # タイムアウト時は停止コマンド送信
            #     cmd = {"linear": 0.0, "angular": 0.0}
            #     self._send_command(cmd)
            
            # 接続切断時の処理
            if self._connection_lost:
                # 緊急停止（可能であれば）
                cmd = {"cmd": "stop"}
                if self._serial_port and self._serial_port.is_open:
                    try:
                        self._send_command(cmd)
                    except Exception:
                        pass  # 送信失敗は無視
                
                # 再接続試行
                if not self._serial_port or not self._serial_port.is_open:
                    self.get_logger().warn('[TB6612] Attempting to reconnect...')
                    try:
                        if self._serial_port:
                            self._serial_port.close()
                        self._connect_serial()
                    except Exception:
                        pass  # 再接続失敗は次回試行
            
        except Exception as e:
            self.get_logger().warn(f'[TB6612] Error in safety check: {e}')
    
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # シリアル接続状態
            serial_status = DiagnosticStatus()
            serial_status.name = "TB6612 Serial Connection"
            serial_status.hardware_id = "tb6612_driver"
            
            if self._connection_lost or not self._serial_port or not self._serial_port.is_open:
                serial_status.level = DiagnosticStatus.ERROR
                serial_status.message = "Serial connection lost"
            elif self._consecutive_errors > 0:
                serial_status.level = DiagnosticStatus.WARN
                serial_status.message = f"Serial errors detected: {self._consecutive_errors}"
            else:
                serial_status.level = DiagnosticStatus.OK
                serial_status.message = "Serial connection healthy"
            
            # パラメータ情報
            serial_status.values.append(KeyValue(
                key="port", value=self.get_parameter('serial_port').value))
            serial_status.values.append(KeyValue(
                key="baud_rate", value=str(self.get_parameter('baud_rate').value)))
            serial_status.values.append(KeyValue(
                key="consecutive_errors", value=str(self._consecutive_errors)))
            
            diag_array.status.append(serial_status)
            
            # オドメトリ状態
            odom_status = DiagnosticStatus()
            odom_status.name = "TB6612 Odometry"
            odom_status.hardware_id = "tb6612_driver"
            
            if self._last_odom_data:
                odom_status.level = DiagnosticStatus.OK
                odom_status.message = "Odometry data available"
                
                # 最新オドメトリ値
                odom_status.values.append(KeyValue(
                    key="x", value=str(self._last_odom_data.get('x', 0))))
                odom_status.values.append(KeyValue(
                    key="y", value=str(self._last_odom_data.get('y', 0))))
                odom_status.values.append(KeyValue(
                    key="angle", value=str(self._last_odom_data.get('a', 0))))
                odom_status.values.append(KeyValue(
                    key="linear_vel", value=str(self._last_odom_data.get('lv', 0))))
                odom_status.values.append(KeyValue(
                    key="angular_vel", value=str(self._last_odom_data.get('av', 0))))
            else:
                odom_status.level = DiagnosticStatus.WARN
                odom_status.message = "No odometry data received"
            
            diag_array.status.append(odom_status)
            
            self._diagnostic_pub.publish(diag_array)
            
        except Exception as e:
            self.get_logger().warn(f'[TB6612] Error publishing diagnostics: {e}')
    
    def destroy_node(self):
        """ノード終了処理"""
        try:
            # 停止コマンド送信
            if self._serial_port and self._serial_port.is_open:
                stop_cmd = {"cmd": "stop"}
                self._send_command(stop_cmd)


            # シリアル接続閉じる
            if self._serial_port:
                self._serial_port.close()
                
            self.get_logger().info('[TB6612] Driver shutdown complete')
            
        except Exception as e:
            self.get_logger().warn(f'[TB6612] Error during shutdown: {e}')
        
        super().destroy_node()




def main(args=None):
    """メイン関数"""
    rclpy.init(args=args)
    
    try:
        driver = TB6612Driver()
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'driver' in locals():
            driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
