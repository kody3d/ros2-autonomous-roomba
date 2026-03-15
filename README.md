# ROS2 Autonomous Roomba 実践構築

ROS2 (Jazzy) を用いた自律走行ロボット。iRobot Create3 をベースに、LiDAR・IMU を統合した SLAM + Nav2 ナビゲーションスタックを構築。実機での完全自律走行を実現。

---

## Demo

**自律走行 — ナビゲーション**

<video src="videos/IMG_3246.mp4" controls width="100%"></video>

**自律走行 — 障害物回避**

<video src="videos/IMG_3329.mp4" controls width="100%"></video>

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
| 計算機 | Raspberry Pi 4 (8GB) |
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