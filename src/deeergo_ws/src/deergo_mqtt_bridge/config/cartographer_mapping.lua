-- ================================================================
-- DeerGo Cartographer 2D - Mapping
--
-- Sensor configuration:
--   LiDAR only
--   no wheel odometry
--   no IMU
--
-- Expected LaserScan:
--   topic    : scan   (launch remaps this to /scan_sync)
--   TF chain : base_link -> <LaserScan frame_id>
--
-- Cartographer provides:
--   map -> odom
--   odom -> base_link
-- ================================================================

include "map_builder.lua"
include "trajectory_builder.lua"


options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- ==============================================================
  -- Frames
  -- ==============================================================

  map_frame = "map",

  -- LiDAR-only 2D tracking.
  tracking_frame = "base_link",

  -- Pose exposed to the rest of ROS.
  published_frame = "base_link",

  -- Continuous local-SLAM frame.
  odom_frame = "odom",

  -- Cartographer itself generates the continuous odom frame.
  provide_odom_frame = true,

  publish_frame_projected_to_2d = true,

  -- ==============================================================
  -- External sensors
  -- ==============================================================

  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,

  -- One sensor_msgs/LaserScan stream.
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- ==============================================================
  -- Timing / publishing
  -- ==============================================================

  lookup_transform_timeout_sec = 0.2,

  submap_publish_period_sec = 0.3,

  pose_publish_period_sec = 0.02,

  trajectory_publish_period_sec = 0.03,

  -- ==============================================================
  -- Sensor sampling
  -- ==============================================================

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}


-- ================================================================
-- 2D trajectory builder
-- ================================================================

MAP_BUILDER.use_trajectory_builder_2d = true


-- ================================================================
-- LiDAR-only
-- ================================================================

TRAJECTORY_BUILDER_2D.use_imu_data = false


-- ================================================================
-- LiDAR range
--
-- UTM-30LX baseline:
--   nominal useful range up to ~30 m.
--
-- IMPORTANT:
-- Check:
--   ros2 topic echo /scan --once
--
-- If the current driver reports a smaller range_max, set max_range
-- to that actual usable value instead.
-- ================================================================

TRAJECTORY_BUILDER_2D.min_range = 0.10

TRAJECTORY_BUILDER_2D.max_range = 30.0

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0


-- ================================================================
-- One complete LaserScan per local-SLAM update
-- ================================================================

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1


-- ================================================================
-- Local scan matching
--
-- No wheel odometry is available, so enable online correlative
-- scan matching to provide a wider local search around the motion
-- extrapolator prediction.
-- ================================================================

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window =
    0.15

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window =
    math.rad(20.0)

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight =
    10.0

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight =
    1.0


-- ================================================================
-- Submaps
-- ================================================================

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05


-- ================================================================
-- Pose graph
-- ================================================================

POSE_GRAPH.optimize_every_n_nodes = 35

POSE_GRAPH.constraint_builder.sampling_ratio = 0.30

POSE_GRAPH.constraint_builder.min_score = 0.55

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60


return options
