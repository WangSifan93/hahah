/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

#pragma once

#include <utility>
#include <vector>

#include "apps/planning/src/common/local_route/local_route.h"
#include "apps/planning/src/planning_msgs/local_route_smoother_config.h"
#include "apps/planning/src/reference_line_provider/smoothers/local_route_smoother.h"

namespace zark {
namespace planning {

class DiscretePointsLocalRouteSmoother : public LocalRouteSmoother {
 public:
  explicit DiscretePointsLocalRouteSmoother(
      const LocalRouteSmootherConfig& config);

  virtual ~DiscretePointsLocalRouteSmoother() = default;

  bool Smooth(const LocalRoute& raw_local_route,
              LocalRoute* const smoothed_lcoal_route,
              const bool is_bound_different = false) override;

  void SetAnchorPoints(const std::vector<RouteAnchorPoint>&);

  void SetLastRefPoints(const std::pair<uint32_t, uint32_t>& ref_index,
                        const std::vector<LocalRoutePoint>& last_ref_points);

  void SetBoundaryFlag(const bool is_boundary) { is_boundary_ = is_boundary; }

 private:
  bool FemPosSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<std::pair<double, double>>& last_raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  bool FemPosSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<std::pair<double, double>>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points,
                       const bool is_zero_point = true);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  bool GenerateRoutePointProfile(
      const LocalRoute& raw_local_route,
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<LocalRoutePoint>* local_route_points);

  std::vector<RouteAnchorPoint> anchor_points_;

  std::vector<LocalRoutePoint> last_ref_points_;

  std::pair<uint32_t, uint32_t> ref_index_;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;

  bool need_change_weight_ = false;

  bool is_boundary_ = false;
};

}  // namespace planning
}  // namespace zark
