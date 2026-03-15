# ROS2 Autonomous Roomba 実践構築(2025 ４-9月構築分)

ROS2 (humble/Jazzy) を用いた自律走行ロボット。iRobot Create3 をベースに、LiDAR・IMU を統合した SLAM + Nav2 ナビゲーションスタックを構築。実機での自律走行を　ROS2学習開始３ヶ月で実現。

2025年4月 ROS2 ロボティクス分野 着手開始
2025年6月 Cartographer と Nav2 を用いた SLAM + ナビゲーションの基本構築完了

2025年6月現在、ROS2 Jazzy での安定動作を確認済み。SLAM とナビゲーションの両方を実装し、障害物回避や狭所走行も可能な高度な自律走行ロボットを構築しています。


---

## Demo

**自律走行 — 障害物回避 ナビゲーション**

<video src="https://github.com/user-attachments/assets/1760a5d6-c8f4-4b87-9cd1-966b20610bd5" controls width="100%"></video>

**自律走行 — 狭所走行**

<video src="https://github.com/user-attachments/assets/f40b25f6-8eaf-464d-89cb-5450d7911521" controls width="100%"></video>

---

## System Architecture

```
┌──────────────────────────────────────────────────────┐
│                    ROS2 Node Graph                   │
│                                                      │
│  YDLiDAR ──► /scan ──► Cartographer (SLAM)          │
│                              │                       │
│  BNO055 ──► /imu ────────────┤                       │
│                              ▼                       │
│  Create3 ──► /odom ──► /map + /tf ──► Nav2 Stack     │
│       ▲                                    │         │
│       └────── /cmd_vel ◄───────────────────┘         │
└──────────────────────────────────────────────────────┘
```

| レイヤ | 実装 |
|---|---|
| SLAM | Cartographer 2D |
| ナビゲーション | Nav2 (DWB controller) |
| センサ融合 | robot_localization (EKF) |
| 通信 | ROS2 DDS (rmw_fastrtps) |

---

## Hardware

| コンポーネント | 型番 |
|---|---|
| ベースロボット | iRobot Create3 |
| 計算機 | Raspberry Pi 4 (4GB) |
| LiDAR | YDLiDAR X4 |
| IMU | BNO055 (9-DOF) |

---

## Software

| カテゴリ | 技術スタック |
|---|---|
| OS / Middleware | Ubuntu 24.04 / ROS2 Jazzy |
| SLAM | Google Cartographer |
| Navigation | Nav2 |
| ドライバ | create3_ros2, ydlidar_ros2_driver, bno055 |
| 言語 | Python 3.12 / C++17 |

---

## How to Run

### 依存関係のインストール

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-cartographer-ros
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### ビルド

```bash
colcon build --symlink-install
source install/setup.bash
```

### 起動

```bash
# SLAM + ナビゲーション
ros2 launch bringup cartlaunchIMU.py

# ジョイスティック操作（手動マッピング時）
ros2 launch launch_joy joy.launch.py
```

---

## Repository Structure

```
src/
├── bringup/          # launch ファイル・Nav2/Cartographer 設定 YAML
├── create_robot/     # iRobot Create3 ドライバ・メッセージ定義
├── create3_nav/      # ナビゲーション補助ノード・地図・URDF
├── bno055/           # BNO055 IMU ドライバ (ROS2)
├── ydlidar_ros2_driver/ # YDLiDAR ドライバ
├── joy_translate/    # ジョイスティック入力 → cmd_vel 変換ノード
├── tb6612_driver/    # TB6612 モータードライバ (拡張用)
└── libcreate/        # iRobot Create C++ ライブラリ
```