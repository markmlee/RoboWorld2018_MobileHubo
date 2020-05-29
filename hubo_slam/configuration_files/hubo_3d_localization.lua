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

include "hubo_3d.lua"

--TRAJECTORY_BUILDER.pure_localization = true
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- For higher resolution uncomment below (Might mean slower execution/lower accuracy)
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.02 --0.15
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.03 --0.1
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20. --20.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.15 --0.45

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes =  4 --100
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1 --0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 4 --10
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options
