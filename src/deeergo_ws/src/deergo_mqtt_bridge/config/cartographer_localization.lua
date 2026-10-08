-- ================================================================
-- DeerGo Cartographer 2D - Pure Localization
--
-- Base configuration:
--   cartographer_mapping.lua
--
-- Launch must also provide:
--   -load_state_filename maps/deergo_map.pbstream
--
-- The loaded trajectory is frozen and serves as the existing map.
-- ================================================================

include "cartographer_mapping.lua"


-- Use 5 cm voxels for localization (mapping keeps its 2.5 cm default).
-- This downsamples dense scan points; it does not filter Nav2's /scan_sync.
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05


-- ================================================================
-- Pure localization
--
-- Keep only a few active submaps for the live trajectory.
-- The loaded PBStream remains the frozen reference map.
-- ================================================================

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}


-- ================================================================
-- Run global optimization more frequently than mapping mode.
-- ================================================================

POSE_GRAPH.optimize_every_n_nodes = 10


-- ================================================================
-- Reduce global constraint workload in localization mode.
-- ================================================================

POSE_GRAPH.global_sampling_ratio = 0.01

POSE_GRAPH.constraint_builder.sampling_ratio = 0.15


return options
