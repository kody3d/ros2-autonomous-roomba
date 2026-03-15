-- Cartographer Configuration for Small Obstacle Detection
-- Roomba + YDLiDAR TMini environment
-- あなたへ　コメントは消さないで


include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",                   -- SLAMで生成されるグローバル座標系
  tracking_frame = "base_link",        -- 通常はIMUやロボットの中心（IMUがbase_linkにマウントされていれば）
  published_frame = "base_footprint",       -- tfで発行されるロボット位置。Navやrvizで使われる
  odom_frame = "odom",                 -- Roombaが発行するオドメトリのフレーム

  provide_odom_frame = false,          -- Cartographerが map→odom のTFを発行するか
  publish_frame_projected_to_2d = true, -- z方向を無視して地面に投影（地図と一致しやすくなる）

  use_odometry = true,                 -- RoombaがOdometry発行するならtrue
  use_nav_sat = false,                 -- GPS非搭載ならfalse
  use_landmarks = false,              -- 特別なランドマーク無しならfalse

  num_laser_scans = 1,                 -- YDLIDARがLaserScan1本発行してるなら1
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1, -- 10Hzスキャンなので1で良い（高周波なら>1）
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.005,
  trajectory_publish_period_sec = 0.03,
  
  -- サンプリング比率パラメータを追加
  rangefinder_sampling_ratio = 1.0,      -- レーザースキャンのサンプリング比率
  odometry_sampling_ratio = 1.0,         -- オドメトリのサンプリング比率
  fixed_frame_pose_sampling_ratio = 1.0, -- 固定フレームポーズのサンプリング比率
  imu_sampling_ratio = 1.0,              -- IMUのサンプリング比率
  landmarks_sampling_ratio = 1.0,        -- ランドマークのサンプリング比率

  -- tf座標の方向を明示的に2D化する（z=0に投影）
  use_pose_extrapolator = true,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D = {
  use_imu_data = false,  -- IMUがあるならtrue（Create 3など）
  use_odometry = false,  -- Roombaがオドメトリを出しているならtrue

  min_range = 0.1,
  max_range = 12.0,
  missing_data_ray_length = 12.0,

  num_accumulated_range_data = 1,  -- 10Hzなら1でOK。30Hzなら3〜5も検討

  voxel_filter_size = 0.025,  -- スキャンを空間的に間引く

  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,               -- 100→200（より多くのポイントを要求）
    max_range = 50.,
  },

  -- リアルタイムスキャンマッチング強化
  use_online_correlative_scan_matching = true,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.15,        -- 0.1→0.15（探索範囲拡大）
    angular_search_window = math.rad(35), -- 30→35度（回転探索拡大）
    translation_delta_cost_weight = 10.0, -- 1.0→10.0（重み強化）
    rotation_delta_cost_weight = 1.0,
  },

  -- Ceresスキャンマッチング強化
  ceres_scan_matcher = {
    occupied_space_weight = 20.0,       -- 1.0→20.0（障害物重視）
    translation_weight = 10.0,
    rotation_weight = 100.0,            -- 40.0→100.0（回転重視）
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,          -- 反復回数増加
      num_threads = 1,
    },
  },

  -- モーションフィルタ（より頻繁なマッチング）
  motion_filter = {
    max_time_seconds = 0.2,             -- 0.5→0.2（頻度向上）
    max_distance_meters = 0.05,         -- 0.1→0.05（細かい動きも記録）
    max_angle_radians = math.rad(2.0),  -- 1.0→2.0度（回転感度向上）
  },

  submaps = {
    num_range_data = 25,                -- 35→25（更新頻度向上）
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05,                -- 適切な解像度
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      insert_free_space = true,
      hit_probability = 0.55,
      miss_probability = 0.45,          -- 0.49→0.45（自由空間判定を強化）
    },
  },
    -- ループクロージャー用アダプティブボクセルフィルタ
  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },
  
}

-- ポーズグラフ最適化設定
-- POSE_GRAPHなしの場合の問題：
-- ループクロージャーが働かない
-- 長時間使用するとマップがズレ続ける
-- 同じ場所を通ってもマップが修正されない
-- 実用的なSLAMには必須です。

-- ポーズグラフ最適化（調整版）
POSE_GRAPH = {
  optimize_every_n_nodes = 40,          -- 90→40（より頻繁な最適化）
  constraint_builder = {
    sampling_ratio = 0.1,               -- 0.03→0.1（サンプリング増加）
    max_constraint_distance = 15.0,
    min_score = 0.50,                   -- 0.55→0.50（より緩い条件）
    global_localization_min_score = 0.55, -- 0.6→0.55（緩和）
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    log_matches = true,
    
    fast_correlative_scan_matcher = {
      linear_search_window = 7.0,
      angular_search_window = math.rad(45.0), -- 30→45度（探索範囲拡大）
      branch_and_bound_depth = 7,
    },
    
    ceres_scan_matcher = {
      occupied_space_weight = 20.0,
      translation_weight = 10.0,
      rotation_weight = 1.0,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,
  
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1.1e2,
    rotation_weight = 1.6e4,
    local_slam_pose_translation_weight = 1e5,
    local_slam_pose_rotation_weight = 1e5,
    odometry_translation_weight = 1e5,   -- オドメトリ重みは高いまま（使用停止だが安全のため）
    odometry_rotation_weight = 1e5,
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 1,                  -- 7→1（リソース軽減）
    },
  },
  
  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10.0,
}

return options