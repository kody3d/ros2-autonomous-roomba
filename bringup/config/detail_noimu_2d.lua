-- Cartographer Configuration for Small Obstacle Detection
-- Roomba + Raspberry Pi 4 environment

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- tracking_frame = "base_link",
  -- published_frame = "base_link",
  tracking_frame = "base_footprint",  -- base_linkからbase_footprintに変更
  published_frame = "base_footprint", -- base_linkからbase_footprintに変更
  
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 2D SLAM用のトラジェクトリビルダー設定
MAP_BUILDER.use_trajectory_builder_2d = true

-- 小さな障害物検出のための重要なパラメータ
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20
-- 値を小さくすると: より頻繁にサブマップ更新、小さな障害物を見逃しにくい
-- 値を大きくすると: サブマップ更新頻度低下、計算負荷軽減

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.02
-- 値を小さくすると: より詳細なマップ、小さな障害物検出向上、メモリ使用量増加
-- 値を大きくすると: 粗いマップ、小さな障害物見逃し、メモリ節約

-- レンジデータ挿入設定（自由空間を白くしやすく調整）
-- 自由空間検出を強化するパラメータ
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"



-- 自由空間記録を有効化（ROS2 Humbleでサポートされている場合のみ）
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.insert_free_space = true

-- miss_probability を小さくして自由空間判定を敏感にする
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.miss_probability = 0.45

-- hit_probability を調整して障害物判定を適度に
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.hit_probability = 0.52

-- センサデータフィルタリング設定
TRAJECTORY_BUILDER_2D.min_range = 0.1
-- 値を小さくすると: より近い障害物も検出、ノイズ増加可能性
-- 値を大きくすると: 近距離ノイズ除去、近い小障害物見逃し

-- TRAJECTORY_BUILDER_2D.max_range = 3.5
-- Roombaの実際のセンサ範囲に合わせる
TRAJECTORY_BUILDER_2D.min_range = 0.08  -- TMiniの最小レンジに合わせて調整（近すぎるデータを除外）
TRAJECTORY_BUILDER_2D.max_range = 4.0  -- TMiniの最大レンジに合わせて調整（遠すぎるデータを除外）

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- スキャンマッチング設定
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres スキャンマッチャー設定
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- モーションフィルタ設定（小さな動きも記録）
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
-- 値を小さくすると: より頻繁にデータ記録、小さな障害物見逃しにくい
-- 値を大きくすると: データ記録頻度低下、処理軽量化

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- アダプティブボクセルフィルタ設定
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.

-- ループクロージャー設定
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.

-- ポーズグラフ最適化設定
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- Fast correlative scan matcher 設定
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres scan matcher for loop closure
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- スキャンマッチングの重みを上げる
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0  -- 増加
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100.0    -- 増加
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400.0       -- 増加

return options