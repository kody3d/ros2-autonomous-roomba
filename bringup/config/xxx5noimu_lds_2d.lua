#Jazzyでは、プランナー（グローバルプランナー）のクラス名が変更されているようです
   # plugins: ["nav2_navfn_planner/NavfnPlanner"]
    # plugins: ["nav2_navfn_planner::NavfnPlanner"]
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"  # base_footprintではなくbase_linkに統一
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: true
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 5000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.1
    recovery_alpha_slow: 0.001
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"  # 完全修飾名を使用
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

    # グローバルローカライゼーションに関するパラメータ
    initial_pose_xx_covariance: 0.5  # x位置の不確かさ
    initial_pose_yy_covariance: 0.5  # y位置の不確かさ
    initial_pose_aa_covariance: 3.14  # 角度の不確かさを大きく設定（約180度）

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link  # base_footprintではなくbase_linkに統一
    odom_topic: /odom
    
    # ComputePathToPoseのエラーを解決するために、最小限の設定に抑える
    navigators:
      - navigate_to_pose
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    
    # 既知のBehavior Treeファイルを使用
    default_bt_xml_filename: "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml"
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # DWB parameters
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.22
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.22
      min_speed_theta: 0.0
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -1.0
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 20
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

planner_server:
  ros__parameters:
    use_sim_time: False
    planner_plugins: ["GridBased"]
    planner_frequency: 5.0  # 20Hzから下げる
    plugins: ["nav2_navfn_planner::NavfnPlanner"]
    
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false  # A*よりもDijkstraの方が計算コストが低い

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "back_up", "wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
    back_up:
      plugin: "nav2_recoveries::BackUp"
    wait:
      plugin: "nav2_recoveries::Wait"
    global_frame: odom
    robot_base_frame: base_link  # base_footprintではなくbase_linkに統一
    transform_timeout: 0.1
    use_sim_time: False
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 0.5

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link  # base_footprintではなくbase_linkに統一
      use_sim_time: False
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.17
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link  # base_footprintではなくbase_linkに統一
      use_sim_time: False
      robot_radius: 0.17
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

collision_monitor:
  ros__parameters:
    use_sim_time: false
    base_frame_id: "base_link"
    global_frame_id: "map"
    transform_tolerance: 0.2
    source_timeout: 1.0
    footprint_topic: "global_costmap/published_footprint"
    
    # 観測ソース設定
    observation_sources: ["scan"]
    scan:
      topic: "/scan"
      data_type: "LaserScan"
      marking: true
      clearing: true
      min_obstacle_height: 0.0
      max_obstacle_height: 1.0
      obstacle_range: 5.0 # 障害物検出の最大距離
      raytrace_range: 6.0
      
    # 衝突回避の設定
    collision_points_topic: "collision_monitor_points"
    polygons: ["collision_polygon"]
    
    # 接近警告の設定（アプローチ）
    collision_polygon:
      type: "polygon"
      points: "[[0.35, 0.35], [0.35, -0.35], [-0.35, -0.35], [-0.35, 0.35]]"  # ロボット周囲の安全領域
      action_type: "approach"
      min_points: 3
      visualize: true
      time_before_collision: 1.2  # 秒数（1.2秒前に警告）
      slowdown_ratio: 0.3  # 減速率
      
    # 通常停止の設定
    # polygon_stop:
    #   type: "polygon"
    #   points: "[[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]]"
    #   action_type: "stop"
    #   min_points: 1
    #   visualize: true
    
    # 緊急停止の設定
    # polygon_slowdown:
    #   type: "polygon"
    #   points: "[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]"
    #   action_type: "slowdown"
    #   min_points: 3
    #   visualize: true
    #   slowdown_ratio: 0.5