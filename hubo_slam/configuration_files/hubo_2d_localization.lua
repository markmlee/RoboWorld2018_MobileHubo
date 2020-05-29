-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "hubo_2d.lua"

-- The number commmented out on the right of each parameter is the default parameter

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

POSE_GRAPH.optimize_every_n_nodes = 1
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 80--1
TRAJECTORY_BUILDER_2D.use_imu_data = true

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2 --10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400 --40

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20--90 size of submaps

POSE_GRAPH.constraint_builder.loop_closure_translation_weight =1e7 --1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight =1e8 --1e5

POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 100--10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 10--1.

POSE_GRAPH.optimization_problem.acceleration_weight = 2e2 --1e3
POSE_GRAPH.optimization_problem.rotation_weight = 3e3 --3e5

-- POSE_GRAPH.constraint_builder.min_score = 0.2--0.55
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.2--0.6

-- POSE_GRAPH.global_sampling_ratio = 0.0001 --0.003
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.01 --0.3
-- POSE_GRAPH.constraint_builder.min_score = 0.8 --0.55
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 15 --7
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.) --30
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 20 --10
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 20 --50

MAP_BUILDER.num_background_threads = 8

-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.55 --0.5
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 180 --200
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 45 --50
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.030 --0.025
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80 --90

return options