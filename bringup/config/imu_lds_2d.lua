include "map_builder.lua"
include "trajectory_builder.lua"

-- Roombaロボットの移動速度（twist）を適切に制限 が望ましい
-- 並進速度: 0.15 m/s以下に制限（max_distance_meters = 0.05, max_time_seconds = 0.3の場合）
-- 回転速度: 0.3 rad/s（約17°/秒）以下に制限（max_angle_radians = 5°, max_time_seconds = 0.3の場合）

-- options: Cartographerの全体的な設定を定義するテーブル
-- マップの座標系、センサーの使用有無、フレーム間の関係などの基本設定を含む
options = {
  map_builder = MAP_BUILDER,  -- マップ構築に関する設定
  trajectory_builder = TRAJECTORY_BUILDER,  -- 軌跡構築に関する設定
  map_frame = "map",  -- マップの座標系
  tracking_frame = "base_link",  -- ロボットの基準フレーム
  published_frame = "odom",  -- Create 2のオドメトリを使用するため
  odom_frame = "odom",  -- オドメトリのフレーム名
  provide_odom_frame = false,  -- オドメトリフレームをCartographerが提供しない
  publish_frame_projected_to_2d = true,  -- 3D→2Dへの投影を有効化
  use_odometry = true,  -- Create 2はオドメトリデータを提供
  use_nav_sat = false,  -- GPSを使用しない
  use_landmarks = false,  -- ランドマークを使用しない
  num_laser_scans = 1,  -- TMiniのスキャンデータを使用
  num_multi_echo_laser_scans = 0,  -- マルチエコースキャンは使用しない
  num_subdivisions_per_laser_scan = 1,  -- レーザースキャンの分割数
  num_point_clouds = 0,  -- ポイントクラウドは使用しない
  lookup_transform_timeout_sec = 0.5,  -- YDLiDARのため少し長めに設定
  submap_publish_period_sec = 0.3,  -- サブマップ公開の頻度
  pose_publish_period_sec = 5e-3,  -- ポーズ公開の頻度
  trajectory_publish_period_sec = 30e-3,  -- 軌跡公開の頻度
  rangefinder_sampling_ratio = 1.,  -- 距離センサーのサンプリング比率
  odometry_sampling_ratio = 1.,  -- オドメトリのサンプリング比率
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定フレームポーズのサンプリング比率
  imu_sampling_ratio = 1.0,  -- IMUありの設定
  landmarks_sampling_ratio = 1.,  -- ランドマークのサンプリング比率
}

MAP_BUILDER.use_trajectory_builder_2d = true  -- 2Dマッピングを使用

-- TRAJECTORY_BUILDER_2D: 2D SLAMの軌跡構築に関する詳細設定
-- スキャンマッチングの調整やモーションフィルタの設定など
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- Create 2はIMUを使用していない
TRAJECTORY_BUILDER_2D.min_range = 0.08  -- TMiniの最小レンジに合わせて調整（近すぎるデータを除外）
TRAJECTORY_BUILDER_2D.max_range = 12.0  -- TMiniの最大レンジに合わせて調整（遠すぎるデータを除外）
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.  -- 欠損データの場合のレイ長（障害物がない場合の扱い）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- オンラインの相関スキャンマッチングを有効化
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1  -- 並進探索範囲を狭める（0.15→0.1）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(45.)  -- 角度ずれ対策 探索範囲を広げる
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0  -- 並進コスト重み
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0  -- 回転コスト重み
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 30.0  -- 占有空間の重みを上げる（20→30）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 15.0  -- 並進の重みを上げる（10→15）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60.0  -- 角度ずれ対策40から60に増加

-- 小型ロボット向けにモーションフィルタを調整
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3  -- 時間閾値を短くする（0.5→0.3）
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05  -- 距離閾値を小さくする（0.1→0.05）
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(3.0)  -- 角度閾値を小さくする（10°→5°→角度ずれ対策3°）

-- サブマップサイズを調整
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 70  -- サブマップあたりのスキャン数を増やす（50→70）
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.02  -- グリッド解像度を上げる 0.05m（5cm->2cm）

-- スキャン歪み対策のパラメータ追加
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.3  -- ボクセルフィルタの最大長（0.5→0.3）
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 250  -- 点数を増やす（200→250）

-- POSE_GRAPH: ループクロージャーと軌跡最適化に関する設定
-- ループの検出条件やグローバル最適化のパラメータを調整
POSE_GRAPH.optimization_problem.huber_scale = 1e2  -- 外れ値に対する堅牢性の調整（Huberロス）
POSE_GRAPH.optimize_every_n_nodes = 30  -- より頻繁に最適化
POSE_GRAPH.constraint_builder.min_score = 0.7  -- 角度ずれ対策0.65->0.7拘束条件とみなす最小スコア
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8  -- 角度ずれ対策0.7->0.8より確実な一致のみを採用 グローバル位置推定の最小スコア
POSE_GRAPH.global_sampling_ratio = 0.003  -- グローバルサンプリング比率
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.  -- グローバル拘束検索を開始する時間

-- Create 2は小型なのでループ検出を調整
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.0  -- 拘束条件を構築する最大距離
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- サンプリング比率
POSE_GRAPH.global_constraint_search_after_n_seconds = 20.  -- グローバル拘束検索を開始する時間（秒）

return options

