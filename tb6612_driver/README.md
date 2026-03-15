# TB6612 ROS2 Driver

TB6612モータードライバボードによる2輪差動駆動ロボット制御のためのROS2ドライバパッケージです。

## 概要

このパッケージは、Raspberry Pi Pico W + TB6612モータードライバによる2輪差動駆動ロボットを制御するためのROS2ドライバです。UART経由でJSON/テキスト形式のコマンド送信と、オドメトリデータの受信を行います。

## 機能

- **速度制御**: `geometry_msgs/Twist`メッセージによる並進・回転速度制御
- **オドメトリ配信**: `nav_msgs/Odometry`メッセージによる自己位置推定データ配信
- **TF配信**: `odom` → `base_footprint`のTF変換配信
- **診断機能**: シリアル接続状態やオドメトリ状態の診断情報配信
- **サービス**: 緊急停止とオドメトリリセットのサービス
- **安全機能**: cmd_velタイムアウト監視、シリアル切断時の緊急停止

## ハードウェア要件

- Raspberry Pi Pico W
- TB6612モータードライバボード
- エンコーダ付きDCモーター × 2
- UART接続（Pi Pico GPIO 16,17 ↔ RasPi GPIO 14,15）

## インストール

```bash
cd ~/ros2_ws/src
git clone [このリポジトリ]
cd ~/ros2_ws
colcon build --packages-select tb6612_driver
source install/setup.bash
```

## 使用方法

### 基本起動

```bash
ros2 launch tb6612_driver tb6612_driver.launch.py
```

### パラメータ指定起動

```bash
ros2 launch tb6612_driver tb6612_driver.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200 \
    max_linear_vel:=0.5 \
    max_angular_vel:=2.0
```

### 速度制御

```bash
# 前進
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'

# 右回転
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: -1.0}}'

# 停止
ros2 topic pub /cmd_vel geometry_msgs/Twist '{}'
```

### サービス

```bash
# 緊急停止
ros2 service call /stop_robot std_srvs/Trigger

# オドメトリリセット
ros2 service call /reset_odometry std_srvs/Trigger
```

## パラメータ

| パラメータ名 | デフォルト値 | 説明 |
|-------------|-------------|------|
| `serial_port` | `/dev/ttyS0` | シリアルポート |
| `baud_rate` | `230400` | ボーレート |
| `base_frame` | `base_footprint` | ロボットベースフレーム |
| `odom_frame` | `odom` | オドメトリフレーム |
| `publish_tf` | `true` | TF配信の有効/無効 |
| `max_linear_vel` | `1.0` | 最大並進速度 [m/s] |
| `max_angular_vel` | `3.14` | 最大回転速度 [rad/s] |
| `cmd_vel_timeout` | `1.0` | cmd_velタイムアウト [秒] |
| `wheel_base` | `0.16` | 車輪間距離 [m] |
| `wheel_radius` | `0.033` | 車輪半径 [m] |

## トピック

### Published

- `/odom` (`nav_msgs/Odometry`): オドメトリデータ
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`): 診断情報
- `/raw_data` (`std_msgs/String`): 生の受信データ

### Subscribed

- `/cmd_vel` (`geometry_msgs/Twist`): 速度コマンド

## サービス

- `/stop_robot` (`std_srvs/Trigger`): 緊急停止
- `/reset_odometry` (`std_srvs/Trigger`): オドメトリリセット

## UART通信プロトコル

### 送信コマンド（JSON形式）

```json
// 速度制御
{"linear": 0.5, "angular": 1.0}

// システムコマンド
{"cmd": "stop"}
{"cmd": "reset_odom"}
{"cmd": "get_status"}
```

### 受信データ（JSON形式）

```json
// オドメトリデータ（20ms間隔）
{
  "x": 1.234,
  "y": 0.567,
  "ang": 0.785,
  "l_vel": 0.500,
  "a_vel": 1.000,
  "enc_a": 1234,
  "enc_b": 5678,
  "send_count": 123,
  "interval_ms": 20.1
}

// 応答データ
{"ack": "set_velocity", "linear": 0.500, "angular": 1.000, "status": "ok"}
```

## 安全機能

1. **cmd_velタイムアウト**: 指定時間内にcmd_velが受信されない場合、自動停止
2. **シリアル切断検出**: 連続エラー回数による切断検出と再接続試行
3. **緊急停止**: 切断時の緊急停止コマンド送信

## 診断情報

- シリアル接続状態
- オドメトリデータ受信状態
- エラー発生回数
- 最新のオドメトリ値

## トラブルシューティング

### シリアル接続エラー

```bash
# ポート権限確認
ls -l /dev/ttyS0
sudo chmod 666 /dev/ttyS0

# 別のポートを試す
ros2 launch tb6612_driver tb6612_driver.launch.py serial_port:=/dev/ttyUSB0
```

### オドメトリデータが受信されない

```bash
# 生データ確認
ros2 topic echo /raw_data

# 診断情報確認
ros2 topic echo /diagnostics
```

## ライセンス

Apache License 2.0
