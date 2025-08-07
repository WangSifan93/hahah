/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

#include "apps/planning/src/reference_line_provider/smoothers/discrete_points_local_route_smoother.h"

#include <algorithm>

#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/common/math/discrete_points_math.h"
#include "apps/planning/src/common/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"
#include "util.h"

namespace zark {
namespace planning {

DiscretePointsLocalRouteSmoother::DiscretePointsLocalRouteSmoother(
    const LocalRouteSmootherConfig& config)
    : LocalRouteSmoother(config) {}

bool DiscretePointsLocalRouteSmoother::Smooth(
    const LocalRoute& raw_local_route, LocalRoute* const smoothed_lcoal_route,
    const bool is_bound_different) {
  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<std::pair<double, double>> last_raw_point2d;
  std::vector<double> anchorpoints_lateralbound;
  std::vector<std::pair<double, double>> diff_anchorpoints_lateralbound;

  if (!is_bound_different) {
    for (const auto& anchor_point : anchor_points_) {
      raw_point2d.emplace_back(anchor_point.path_point.x(),
                               anchor_point.path_point.y());
      anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
    }
    if (anchorpoints_lateralbound.size() > 0) {
      // fix front and back points to avoid end states deviate from the center
      // of road
      anchorpoints_lateralbound.front() = 0.0;
      anchorpoints_lateralbound.back() = 0.0;
    }
  } else {
    for (const auto& anchor_point : anchor_points_) {
      raw_point2d.emplace_back(anchor_point.path_point.x(),
                               anchor_point.path_point.y());
      diff_anchorpoints_lateralbound.emplace_back(std::make_pair<>(
          anchor_point.lateral_bound, anchor_point.lateral_low_bound));
    }
  }

  for (const auto& last_ref_point : last_ref_points_) {
    last_raw_point2d.emplace_back(last_ref_point.x(), last_ref_point.y());
  }

  NormalizePoints(&raw_point2d);
  if (last_raw_point2d.size() > 0) {
    NormalizePoints(&last_raw_point2d, false);
  }
  bool status = false;

  std::vector<std::pair<double, double>> smoothed_point2d;
  if (!is_bound_different) {
    status = FemPosSmooth(raw_point2d, last_raw_point2d,
                          anchorpoints_lateralbound, &smoothed_point2d);
  } else {
    status = FemPosSmooth(raw_point2d, diff_anchorpoints_lateralbound,
                          &smoothed_point2d);
  }

  if (!status) {
    AERROR << "discrete_points local route smoother fails";
    return false;
  }

  DeNormalizePoints(&smoothed_point2d);

  std::vector<LocalRoutePoint> route_points;
  if (!is_boundary_) {
    GenerateRoutePointProfile(raw_local_route, smoothed_point2d, &route_points);
  } else {
    for (const auto& point : smoothed_point2d) {
      route_points.emplace_back(LocalRoutePoint(
          hdmap::MapPathPoint(::math::Vec2d(point.first, point.second), 0.0),
          0.0, 0.0));
    }
  }

  LocalRoutePoint::RemoveDuplicates(&route_points);

  if (route_points.size() < 2) {
    AERROR << "Fail to generate smoothed local route.";
    return false;
  }

  *smoothed_lcoal_route = LocalRoute(route_points, raw_local_route.GetConfig());
  return true;
}

bool DiscretePointsLocalRouteSmoother::FemPosSmooth(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<std::pair<double, double>>& last_raw_point2d,
    const std::vector<double>& bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  const auto& fem_pos_config =
      config_.discrete_points().fem_pos_deviation_smoothing();

  FemPosDeviationSmoother smoother(fem_pos_config);

  // box contraints on pos are used in fem pos smoother, thus shrink the
  // bounds by 1.0 / sqrt(2.0)
  std::vector<double> box_bounds = bounds;
  const double box_ratio = 1.0 / std::sqrt(2.0);
  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  std::vector<double> opt_x;
  std::vector<double> opt_y;
  bool status = smoother.Solve(raw_point2d, last_raw_point2d, ref_index_,
                               box_bounds, &opt_x, &opt_y);

  if (!status) {
    AERROR << "Fem Pos local route smoothing failed";
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    AERROR << "Return by fem pos smoother is wrong. Size smaller than 2 ";
    return false;
  }

  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

  size_t point_size = opt_x.size();
  for (size_t i = 0; i < point_size; ++i) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

bool DiscretePointsLocalRouteSmoother::FemPosSmooth(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<std::pair<double, double>>& bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  const auto& fem_pos_config =
      config_.discrete_points().fem_pos_deviation_smoothing();
  FemPosDeviationSmoother smoother(fem_pos_config);
  std::vector<double> opt_x;
  std::vector<double> opt_y;
  bool status = smoother.Solve(raw_point2d, bounds, &opt_x, &opt_y);

  if (!status) {
    AERROR << "Fem Pos reference line smoothing failed";
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    AERROR << "Return by fem pos smoother is wrong. Size smaller than 2 ";
    return false;
  }

  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

  size_t point_size = opt_x.size();
  for (size_t i = 0; i < point_size; ++i) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

void DiscretePointsLocalRouteSmoother::SetAnchorPoints(
    const std::vector<RouteAnchorPoint>& anchor_points) {
  CHECK_GT(anchor_points.size(), 1U);
  anchor_points_ = anchor_points;
  need_change_weight_ = false;
  is_boundary_ = false;
  for (const auto& point : anchor_points) {
    if (point.need_change_weight) {
      need_change_weight_ = true;
    }
    break;
  }
  last_ref_points_.clear();
  ref_index_ = {0, 0};
}

void DiscretePointsLocalRouteSmoother::SetLastRefPoints(
    const std::pair<uint32_t, uint32_t>& index,
    const std::vector<LocalRoutePoint>& last_ref_points) {
  last_ref_points_ = last_ref_points;
  ref_index_ = index;
}

void DiscretePointsLocalRouteSmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points,
    const bool is_zero_point) {
  if (is_zero_point) {
    zero_x_ = xy_points->front().first;
    zero_y_ = xy_points->front().second;
  }
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

void DiscretePointsLocalRouteSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

bool DiscretePointsLocalRouteSmoother::GenerateRoutePointProfile(
    const LocalRoute& raw_local_route,
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<LocalRoutePoint>* local_route_points) {
  // Compute path profile
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!DiscretePointsMath::ComputePathProfile(
          xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }

  // Load into ReferencePoints
  size_t points_size = xy_points.size();
  for (size_t i = 0; i < points_size; ++i) {
    ::common::SLPoint ref_sl_point;
    if (!raw_local_route.XYToSL({xy_points[i].first, xy_points[i].second},
                                ref_sl_point)) {
      return false;
    }
    const double kEpsilon = 1e-6;
    if (ref_sl_point.s() < -kEpsilon ||
        ref_sl_point.s() > raw_local_route.Length()) {
      continue;
    }
    ref_sl_point.set_s(std::max(ref_sl_point.s(), 0.0));
    LocalRoutePoint rlp = raw_local_route.GetLocalRoutePoint(ref_sl_point.s());
    auto new_lane_waypoints = rlp.lane_waypoints();
    for (auto& lane_waypoint : new_lane_waypoints) {
      lane_waypoint.l = ref_sl_point.l();
    }
    local_route_points->emplace_back(LocalRoutePoint(
        hdmap::MapPathPoint(
            ::math::Vec2d(xy_points[i].first, xy_points[i].second), headings[i],
            new_lane_waypoints),
        kappas[i], dkappas[i]));
  }
  return true;
}

}  // namespace planning
}  // namespace zark
