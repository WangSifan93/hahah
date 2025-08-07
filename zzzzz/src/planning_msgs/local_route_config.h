/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <string>

#include "apps/planning/src/planning_msgs/local_route_smoother_config.h"
#include "apps/planning/src/planning_msgs/acc_route_config.h"

namespace zark {
namespace planning {

struct LocalRouteConfig {
 public:
  bool use_map_state;
  bool self_test_local_route;
  double local_route_speed_limit;
  double min_local_route_speed_limit;
  int16_t local_route_thread_time;
  bool enable_local_route_stitching;
  bool enable_smooth_local_route;
  double look_forward_extend_distance;
  double local_route_stitch_overlap_distance;
  double default_local_route_width;
  double smoothed_local_route_max_diff;
  double default_lane_width;
  double look_backward_distance;
  double look_forward_short_distance;
  double look_forward_long_distance;
  double obs_filter_front_start_s;
  double obs_filter_front_end_s;
  double obs_filter_rear_start_s;
  double obs_filter_rear_end_s;
  double look_forward_time_sec;
  double adc_width;
  uint8_t fit_points_size;
  double noa_line_forward_dis;
  bool offline_pack;
  float keep_trigger_time;
  float latral_select_distance;
  float NoaLine_section_length;
  float max_lane_width;
  float normal_lane_width;
  LocalRouteSmootherConfig smoother_config;
  ACCRouteConfig acc_route_config;
};

}  // namespace planning
}  // namespace zark
