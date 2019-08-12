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

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 15

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.

POSE_GRAPH.optimization_problem.huber_scale = 1e2
-----------------TUNE THESE PARAMETERS FOR LOW LATENCY-------------------------------

------------Global SLAM------------
POSE_GRAPH.optimize_every_n_nodes = 100 -- Decrease
POSE_GRAPH.global_sampling_ratio = 0.003 -- Decrease
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4 -- Decrease
POSE_GRAPH.constraint_builder.min_score = 0.85 -- Increase
POSE_GRAPH.global_constraint_search_after_n_seconds = 30 -- Increase

---------Global/Local SLAM---------
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100 -- Decrease
TRAJECTORY_BUILDER_2D.max_range = 10. -- Decrease

-------------------------------------------------------------------------------------
return options
