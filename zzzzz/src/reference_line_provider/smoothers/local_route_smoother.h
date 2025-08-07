//
// Copyright 2024 zpilot. All Rights Reserved.
//

/**
 * @file
 **/

#pragma once

#include <vector>

#include "apps/planning/src/common/local_route/local_route.h"
#include "apps/planning/src/planning_msgs/local_route_smoother_config.h"

namespace zark {
namespace planning {

struct RouteAnchorPoint {
  ::common::PathPoint path_point;
  double lateral_bound = 0.0;
  double lateral_low_bound = 0.0;
  double longitudinal_bound = 0.0;
  // enforce smoother to strictly follow this route point
  bool enforced = false;
  bool diff_bound = false;          // diff lateral up and low bound
  bool need_change_weight = false;  // change fem and ref weight
};

class LocalRouteSmoother {
 public:
  explicit LocalRouteSmoother(const LocalRouteSmootherConfig& config)
      : config_(config) {}

  /**
   * Smoothing constraints
   */
  virtual void SetAnchorPoints(
      const std::vector<RouteAnchorPoint>& achor_points) = 0;

  virtual void SetLastRefPoints(
      const std::pair<uint32_t, uint32_t>& ref_index,
      const std::vector<LocalRoutePoint>& last_ref_points) = 0;

  virtual void SetBoundaryFlag(const bool is_boundary) = 0;

  /**
   * Smooth a given local route
   */
  virtual bool Smooth(const LocalRoute&, LocalRoute* const,
                      const bool is_different_bound = false) = 0;

  virtual ~LocalRouteSmoother() = default;

 protected:
  LocalRouteSmootherConfig config_;
};

}  // namespace planning
}  // namespace zark
