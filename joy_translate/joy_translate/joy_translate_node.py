import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import subprocess
import os
from pathlib import Path

#  これはJoystick　F310から
#  roomba create3
#  へのtwist変換である

# axes[0]: 左スティック 左右 (左が正、右が負)
# axes[1]: 左スティック 前後 (前が正、後ろが負) ← 現在の線形速度(x)制御
# axes[2]: 左トリガー (デフォルト1.0、押すと-1.0)
# axes[3]: 右スティック 左右 (左が正、右が負) ← 現在の角速度(z)制御
# axes[4]: 右スティック 前後 (前が正、後ろが負)
# axes[5]: 右トリガー (デフォルト1.0、押すと-1.0)
# axes[6]: 十字キー 左右 (左1.0、右-1.0)
# axes[7]: 十字キー 上下 (上1.0、下-1.0)
                  
class JoyTranslate(Node): 
    def __init__(self):
        super().__init__('joy_translate_node') 
        
        # 起動時にコントローラー接続スクリプトを実行
        self.connect_controller()
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        
        # 現在の速度と目標速度を別々に管理
        self.current_vel = Twist()
        self.target_vel = Twist()
        
        # 減衰係数 (0.0〜1.0): 小さいほど滑らかになるが反応は遅くなる
        self.smoothing_factor = 0.25  # 0.2から0.15に下げて加速をより滑らかに
        
        # 速度の最大値を設定
        self.max_linear_speed = 0.15  # 最大直進速度 (m/s)
        self.max_angular_speed = 0.3  # 最大回転速度 (rad/s)
        
        # ログ出力のカウンター (最初の10回だけログを出力)
        self.log_counter = 0
        self.max_logs = 10
        
        # 速度更新用タイマー (20Hzで更新)
        self.timer = self.create_timer(0.05, self.update_velocity)
        
        # ジョイスティック接続確認用タイマー (30秒ごとに確認)
        self.joy_check_timer = self.create_timer(30.0, self.check_joystick_connection)
        
        # 最後にジョイスティックからメッセージを受信した時間
        self.last_joy_msg_time = self.get_clock().now()
        
        self.get_logger().info('Joy translate node initialized with controller connection')

    def connect_controller(self):
        """コントローラー接続スクリプトを実行する"""
        try:
            # スクリプトのパスを設定
            home_dir = str(Path.home())
            script_path = os.path.join(home_dir, 'create_ws/connect_controller.sh')
            
            # スクリプトが存在するか確認
            if not os.path.exists(script_path):
                self.get_logger().error(f'Controller connection script not found at: {script_path}')
                return
            
            # スクリプトを非同期で実行（バックグラウンドプロセス）
            self.get_logger().info('Attempting to connect controller...')
            process = subprocess.Popen(
                [script_path], 
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # ログ出力用のスレッドを作成
            def log_output():
                for line in process.stdout:
                    self.get_logger().info(f'Controller script: {line.strip()}')
                for line in process.stderr:
                    self.get_logger().error(f'Controller script error: {line.strip()}')
            
            # スレッドを開始（非同期処理）
            import threading
            thread = threading.Thread(target=log_output)
            thread.daemon = True  # メインプログラム終了時にスレッドも終了
            thread.start()
            
            self.get_logger().info('Controller connection script started')
            
        except Exception as e:
            self.get_logger().error(f'Failed to run controller connection script: {str(e)}')

    def check_joystick_connection(self):
        """ジョイスティック接続が切れていないか確認"""
        now = self.get_clock().now()
        # 最後のジョイスティックメッセージから60秒以上経過している場合、再接続を試みる
        if (now - self.last_joy_msg_time).nanoseconds / 1e9 > 60.0:
            self.get_logger().warn('No joystick messages received for 60 seconds. Attempting to reconnect...')
            self.connect_controller()

    def listener_callback(self, joy_msg): 
        # ジョイスティックメッセージ受信時間を更新
        self.last_joy_msg_time = self.get_clock().now()
        
        # 左右のスティックどちらでも操作できるようにする
        # 前後移動：左スティック前後(axes[1])または右スティック前後(axes[4])
        # 回転：左スティック左右(axes[0])または右スティック左右(axes[3])
        
        # 左右のスティックで大きい値を採用
        linear_left = joy_msg.axes[1]
        linear_right = joy_msg.axes[4]
        angular_left = joy_msg.axes[0]
        angular_right = joy_msg.axes[3]
        
        # 絶対値が大きい方を採用
        linear_input = linear_left if abs(linear_left) > abs(linear_right) else linear_right
        angular_input = angular_left if abs(angular_left) > abs(angular_right) else angular_right
        
        # 最大速度を適用
        self.target_vel.linear.x = linear_input * self.max_linear_speed
        
        # 後退時（線形速度が負の場合）に左右を反転させる
        # これにより、後退時の回転方向が前進時と同じになる
  
        # if self.target_vel.linear.x < 0:
        #     # 後退時は左右反転
        #     self.target_vel.angular.z = -angular_input * self.max_angular_speed
        # else:
            # 前進時は通常通り
        self.target_vel.angular.z = angular_input * self.max_angular_speed
        
        self.get_logger().debug(f"Target velocity: Linear={self.target_vel.linear.x:.2f}, Angular={self.target_vel.angular.z:.2f}")

    def update_velocity(self):
        # 現在の速度を目標速度に徐々に近づける
        self.current_vel.linear.x += self.smoothing_factor * (self.target_vel.linear.x - self.current_vel.linear.x)
        self.current_vel.angular.z += self.smoothing_factor * (self.target_vel.angular.z - self.current_vel.angular.z)
        
        # 非常に小さい値は0とみなす（ドリフト防止）
        if abs(self.current_vel.linear.x) < 0.01:
            self.current_vel.linear.x = 0.0
        if abs(self.current_vel.angular.z) < 0.01:
            self.current_vel.angular.z = 0.0
        
        # 更新された速度を発行
        self.publisher.publish(self.current_vel)
        
        # 最初の10回だけログを出力
        if self.log_counter < self.max_logs:
            self.get_logger().info(f"Current: Linear={self.current_vel.linear.x:.2f}, Angular={self.current_vel.angular.z:.2f}")
            self.log_counter += 1

def main(args=None):
    rclpy.init(args=args)
    joy_translate = JoyTranslate()
    rclpy.spin(joy_translate)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
Raspi 4 ubuntu 22.04への BLE対応づけ
sudo vi /boot/firmware/config.txt
dtoverlay=miniuart-bt
を追加して
reboot
bluetoothctl を起動
bluetoothctl
すると対話モードになります：
[bluetooth]#
Bluetoothモード設定（順番が大事）
[bluetooth]# power on
[bluetooth]# agent on
[bluetooth]# default-agent
[bluetooth]# scan on
PS4コントローラーをペアリングモードにする
SHAREボタンとPSボタンを同時に長押し（白色LEDが点滅するまで）。
すると bluetoothctl 上で見つかるはずです：
[NEW] Device 2F:3E:59:7D:42:8A Wireless Controller
pair 2F:3E:59:7D:42:8A
trust 2F:3E:59:7D:42:8A
connect 2F:3E:59:7D:42:8A
scan を停止して終了
scan off
exit
bluetoothctl info 2F:3E:59:7D:42:8A
Device 2F:3E:59:7D:42:8A (public)
	Name: Wireless Controller
	Alias: Wireless Controller
	Class: 0x00002508
	Icon: input-gaming
	Paired: yes
	Trusted: yes
	Blocked: no
	Connected: yes
	WakeAllowed: yes
	LegacyPairing: no
	UUID: Human Interface Device... (00001124-0000-1000-8000-00805f9b34fb)
	UUID: PnP Information      (00001200-0000-1000-8000-00805f9b34fb)
	Modalias: usb:v054Cp09CCd0100
sudo evtest
No device specified, trying to scan all of /dev/input/event*
Available devices:
/dev/input/event1:	vc4-hdmi-0
/dev/input/event2:	vc4-hdmi-1
/dev/input/event3:	Wireless Controller Touchpad
/dev/input/event4:	Wireless Controller Motion Sensors
/dev/input/event5:	Wireless Controller 
"""

