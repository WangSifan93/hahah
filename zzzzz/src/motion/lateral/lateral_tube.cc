/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_tube.cc
 **/

#include "apps/planning/src/motion/lateral/lateral_tube.h"
#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include <optional>

namespace zark {
namespace planning {

constexpr double kDefaultLaneHalfWidth = 1.75;  // [m]

Tube LateralTube::ConstructTubes(
    const ::common::TrajectoryPoint& planning_start_point,
    const std::vector<Nudger>& nudgers, const CorridorInfo& corridor_info,
    const LonMPCData& lon_mpc_data, const Corridor& corridor_prev,
    const std::vector<Tube::TubePoint>& tube_prev,
    const LateralLookupTables& lateral_tables,
    const LateralPadding& lateral_padding,
    const STProposal& st_proposal,const LocalRoute& cur_local_route) const {
  const int n_steps = config_.model.num_steps;
  const int n_nodes = n_steps + 1;
  const double dt = config_.model.dt;
  const Corridor& corridor = corridor_info.GetCorridor();
  const double soft_padding =
      lateral_padding.GetSoftPadding(lateral_tables, planning_start_point.v());
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double adc_half_width = vehicle_param.width() / 2.0;
  std::optional<STPoint> bottom_left_point = FindBottomLeftPoint(st_proposal);

  Tube tube;
  tube.pts.reserve(n_nodes);
  for (int k = 0; k < n_nodes; k++) {
    const CorridorPoint corridor_point =
        corridor.EvaluateByS(lon_mpc_data.x.coeff(kIdxS, k));
    const double stiff_left_padding = lateral_padding.GetStiffPadding(
        lateral_tables, perception::ST_UNKNOWN,
        corridor_point.type_left == CorridorPoint::Type::CURB);
    const double stiff_right_padding = lateral_padding.GetStiffPadding(
        lateral_tables, perception::ST_UNKNOWN,
        corridor_point.type_right == CorridorPoint::Type::CURB);

    Tube::TubePoint tube_point;
    tube_point.t = lon_mpc_data.t.coeff(k);
    tube_point.s = lon_mpc_data.x.coeff(kIdxS, k);
    double lanes_left_boundary = kDefaultLaneHalfWidth;
    double lanes_right_boundary = -kDefaultLaneHalfWidth;
    GetCorridorAxisLanesWidth(corridor_point, cur_local_route, lanes_left_boundary,
                  lanes_right_boundary);
    if (k == 0 && !tube_prev.empty()) {
      ::common::SLPoint tube_sl_point_prev;
      corridor_prev.XYToSL(::math::Vec2d(planning_start_point.path_point().x(),
                                         planning_start_point.path_point().y()),
                           tube_sl_point_prev);
      Tube::TubePoint tube_start_point =
          EvaluateByS(tube_prev, tube_sl_point_prev.s());
      ::common::SLPoint left_sl_point, right_sl_point;
      corridor.XYToSL(tube_start_point.xy_left_ref, left_sl_point);
      corridor.XYToSL(tube_start_point.xy_right_ref, right_sl_point);
      tube_point.l_left_ref = left_sl_point.l();
      tube_point.l_right_ref = right_sl_point.l();
    } else {
      tube_point.l_left_ref =
          std::min(corridor_point.l + adc_half_width, lanes_left_boundary);
      tube_point.l_right_ref =
          std::max(corridor_point.l - adc_half_width, lanes_right_boundary);
    }
    tube_point.l_left_hard = std::min(corridor_point.l_left, lanes_left_boundary);
    tube_point.l_left_stiff = tube_point.l_left_hard - stiff_left_padding;
    tube_point.l_left_soft = tube_point.l_left_stiff - soft_padding;
    tube_point.l_right_hard = std::max(corridor_point.l_right, lanes_right_boundary);
    tube_point.l_right_stiff = tube_point.l_right_hard + stiff_right_padding;

    tube_point.l_right_soft = tube_point.l_right_stiff + soft_padding;
    tube.pts.emplace_back(tube_point);
  }
  for (int i = 0; i < static_cast<int>(nudgers.size()); i++) {
    for (int k = 0; k < n_nodes; k++) {
      if (std::isnan(nudgers[i].l[k]) || std::isnan(nudgers[i].v[k])) {
        continue;
      }
      if (bottom_left_point.has_value() && (k * dt > bottom_left_point->t() &&
          lon_mpc_data.x.coeff(kIdxS, k) > bottom_left_point->s())) {
        continue;
      }

      const CorridorPoint corridor_point =
          corridor.EvaluateByS(lon_mpc_data.x.coeff(0, k));
      const double stiff_left_padding = lateral_padding.GetStiffPadding(
          lateral_tables, perception::ST_UNKNOWN,
          corridor_point.type_left == CorridorPoint::Type::CURB);
      const double stiff_right_padding = lateral_padding.GetStiffPadding(
          lateral_tables, perception::ST_UNKNOWN,
          corridor_point.type_right == CorridorPoint::Type::CURB);
      const double soft_padding_speed_zero =
          lateral_padding.GetSoftPadding(lateral_tables, 0.0);
      if (nudgers[i].is_left) {
        const double l_left_max = corridor_point.l_right + stiff_right_padding +
                                  soft_padding_speed_zero +
                                  vehicle_param.width();
        tube.pts[k].l_left_hard = std::min(
            std::max(nudgers[i].l[k], l_left_max), tube.pts[k].l_left_hard);
        tube.pts[k].l_left_stiff =
            std::min(std::max(nudgers[i].l[k] - stiff_left_padding, l_left_max),
                     tube.pts[k].l_left_stiff);
        tube.pts[k].l_left_soft = std::min(
            std::max(nudgers[i].l[k] - stiff_left_padding - soft_padding,
                     l_left_max),
            tube.pts[k].l_left_soft);
      } else {
        const double l_right_min = corridor_point.l_left - stiff_left_padding -
                                   soft_padding_speed_zero -
                                   vehicle_param.width();
        tube.pts[k].l_right_hard = std::max(
            std::min(nudgers[i].l[k], l_right_min), tube.pts[k].l_right_hard);
        tube.pts[k].l_right_stiff = std::max(
            std::min(nudgers[i].l[k] + stiff_right_padding, l_right_min),
            tube.pts[k].l_right_stiff);
        tube.pts[k].l_right_soft = std::max(
            std::min(nudgers[i].l[k] + stiff_right_padding + soft_padding,
                     l_right_min),
            tube.pts[k].l_right_soft);
      }
    }
  }

  for (int k = 0; k < n_nodes; k++) {
    const CorridorPoint corridor_point =
        corridor.EvaluateByS(lon_mpc_data.x.coeff(0, k));
    tube.pts[k].l_left_ref =
        std::min(tube.pts[k].l_left_soft, tube.pts[k].l_left_ref);
    tube.pts[k].l_right_ref =
        std::max(tube.pts[k].l_right_soft, tube.pts[k].l_right_ref);
    const double forced_left_dis_for_right =
        corridor_point.l - adc_half_width -
        (corridor_point.l + adc_half_width - tube.pts[k].l_left_ref);
    const double forced_right_dis_for_left =
        corridor_point.l + adc_half_width +
        (tube.pts[k].l_right_ref - (corridor_point.l - adc_half_width));
    tube.pts[k].l_left_ref =
        std::max(forced_right_dis_for_left, tube.pts[k].l_left_ref);
    tube.pts[k].l_right_ref =
        std::min(forced_left_dis_for_right, tube.pts[k].l_right_ref);
  }

  SmoothTubeReference(tube.pts, true);
  SmoothTubeReference(tube.pts, false);
  ConvertVec2dPoint(tube, corridor);
  ADEBUG << "tube points size : " << tube.pts.size();
  return tube;
}

void LateralTube::ConvertVec2dPoint(Tube& tube,
                                    const Corridor& corridor) const {
  for (int k = 0; k < static_cast<int>(tube.pts.size()); k++) {
    Tube::TubePoint& pt = tube.pts[k];
    if (!corridor.SLToXY(
            ::common::SLPoint(tube.pts[k].s, tube.pts[k].l_left_ref),
            &pt.xy_left_ref)) {
      AERROR << "Failed to convert from SL to XY for l_left_ref";
    };
    if (!corridor.SLToXY(
            ::common::SLPoint(tube.pts[k].s, tube.pts[k].l_right_ref),
            &pt.xy_right_ref)) {
      AERROR << "Failed to convert from SL to XY for l_right_ref";
    };
    if (!corridor.SLToXY(
            ::common::SLPoint(tube.pts[k].s, tube.pts[k].l_left_soft),
            &pt.xy_left_soft)) {
      AERROR << "Failed to convert from SL to XY for l_left_soft";
    };
    if (!corridor.SLToXY(
            ::common::SLPoint(tube.pts[k].s, tube.pts[k].l_right_soft),
            &pt.xy_right_soft)) {
      AERROR << "Failed to convert from SL to XY for l_right_soft";
    };
    if (!corridor.SLToXY(
            ::common::SLPoint(tube.pts[k].s, tube.pts[k].l_left_stiff),
            &pt.xy_left_stiff)) {
      AERROR << "Failed to convert from SL to XY for l_left_stiff";
    };
    if (!corridor.SLToXY(
            ::common::SLPoint(tube.pts[k].s, tube.pts[k].l_right_stiff),
            &pt.xy_right_stiff)) {
      AERROR << "Failed to convert from SL to XY for l_right_stiff";
    };
    if (!corridor.SLToXY(
            ::common::SLPoint(tube.pts[k].s, tube.pts[k].l_left_hard),
            &pt.xy_left_hard)) {
      AERROR << "Failed to convert from SL to XY for l_left_hard";
    };
    if (!corridor.SLToXY(
            ::common::SLPoint(tube.pts[k].s, tube.pts[k].l_right_hard),
            &pt.xy_right_hard)) {
      AERROR << "Failed to convert from SL to XY for l_right_hard";
    };
  }
}

void LateralTube::SmoothTubeReference(std::vector<Tube::TubePoint>& pts,
                                      const bool is_left) const {
  const int n_steps = config_.model.num_steps;
  const int n_nodes = n_steps + 1;
  const double dt = config_.model.dt;
  const double dl_dt = config_.ref.dl_dt;
  if (is_left) {
    // left forward pass
    for (int k = 0; k < n_nodes - 1; ++k) {
      const double l_ref_curr = pts[k].l_left_ref;
      double l_ref_next = pts[k + 1].l_left_ref;
      if (l_ref_next > l_ref_curr) {
        pts[k + 1].l_left_ref = std::min(l_ref_next, l_ref_curr + dl_dt * dt);
      }
    }
    // left backward pass
    for (int k = n_nodes - 1; k > 0; --k) {
      const double l_ref_curr = pts[k].l_left_ref;
      double l_ref_prev = pts[k - 1].l_left_ref;
      if (l_ref_prev > l_ref_curr) {
        pts[k - 1].l_left_ref =
            std::min(pts[k - 1].l_left_ref, l_ref_curr + dl_dt * dt);
      }
    }
  } else {
    // right forward pass
    for (int k = 0; k < n_nodes - 1; ++k) {
      const double l_ref_curr = pts[k].l_right_ref;
      double l_ref_next = pts[k + 1].l_right_ref;
      if (l_ref_next < l_ref_curr) {
        pts[k + 1].l_right_ref =
            std::max(pts[k + 1].l_right_ref, l_ref_curr - dl_dt * dt);
      }
    }
    // right backward pass
    for (int k = n_nodes - 1; k > 0; --k) {
      const double l_ref_curr = pts[k].l_right_ref;
      double l_ref_prev = pts[k - 1].l_right_ref;
      if (l_ref_prev < l_ref_curr) {
        pts[k - 1].l_right_ref =
            std::max(pts[k - 1].l_right_ref, l_ref_curr - dl_dt * dt);
      }
    }
  }
}

Tube::TubePoint LateralTube::EvaluateByS(
    const std::vector<Tube::TubePoint>& tube, const double s) const {
  auto compare_s = [](const Tube::TubePoint& point, const double s_ref) {
    return point.s < s_ref;
  };
  auto it_lower = std::lower_bound(tube.begin(), tube.end(), s, compare_s);

  ::common::SLPoint sl_point;
  if (it_lower == tube.end()) {
    return tube.back();
  } else if (it_lower == tube.begin()) {
    return tube.front();
  } else {
    Tube::TubePoint tube_point;
    const double kNumMin = 1e-3;
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    if (p1.s - p0.s < kNumMin) {  // return when s is too close
      return p0;                  // or s is decreasing
    }
    const double ratio = (s - p0.s) / (p1.s - p0.s);

    tube_point.s = s;
    tube_point.t = ::math::lerp(p0.t, p0.s, p1.t, p1.s, s);
    tube_point.l_left_ref =
        ::math::lerp(p0.l_left_ref, p0.s, p1.l_left_ref, p1.s, s);
    tube_point.l_right_ref =
        ::math::lerp(p0.l_right_ref, p0.s, p1.l_right_ref, p1.s, s);
    tube_point.l_left_soft =
        ::math::lerp(p0.l_left_soft, p0.s, p1.l_left_soft, p1.s, s);
    tube_point.l_right_soft =
        ::math::lerp(p0.l_right_soft, p0.s, p1.l_right_soft, p1.s, s);
    tube_point.l_left_stiff =
        ::math::lerp(p0.l_left_stiff, p0.s, p1.l_left_stiff, p1.s, s);
    tube_point.l_right_stiff =
        ::math::lerp(p0.l_right_stiff, p0.s, p1.l_right_stiff, p1.s, s);
    tube_point.l_left_hard =
        ::math::lerp(p0.l_left_hard, p0.s, p1.l_left_hard, p1.s, s);
    tube_point.l_right_hard =
        ::math::lerp(p0.l_right_hard, p0.s, p1.l_right_hard, p1.s, s);

    tube_point.xy_left_ref =
        p1.xy_left_ref * ratio + p0.xy_left_ref * (1 - ratio);
    tube_point.xy_right_ref =
        p1.xy_right_ref * ratio + p0.xy_right_ref * (1 - ratio);
    tube_point.xy_left_soft =
        p1.xy_left_soft * ratio + p0.xy_left_soft * (1 - ratio);
    tube_point.xy_right_soft =
        p1.xy_right_soft * ratio + p0.xy_right_soft * (1 - ratio);
    tube_point.xy_left_stiff =
        p1.xy_left_stiff * ratio + p0.xy_left_stiff * (1 - ratio);
    tube_point.xy_right_stiff =
        p1.xy_right_stiff * ratio + p0.xy_right_stiff * (1 - ratio);
    tube_point.xy_left_hard =
        p1.xy_left_hard * ratio + p0.xy_left_hard * (1 - ratio);
    tube_point.xy_right_hard =
        p1.xy_right_hard * ratio + p0.xy_right_hard * (1 - ratio);
    return tube_point;
  }
}

std::optional<STPoint> LateralTube::FindBottomLeftPoint(
    const STProposal& st_proposal) const {
  if (st_proposal.empty()) {
    return std::nullopt;
  }
  std::optional<STPoint> bottom_left_point = std::nullopt;
  for (const auto& [st_boundary, is_front, is_filtered] : st_proposal) {
    const auto& lower_points = st_boundary->lower_points();
    if (is_front && !lower_points.empty()) {
      for (const auto& point : lower_points) {
        if (!bottom_left_point || point.s() < bottom_left_point->s() ||
            (point.s() == bottom_left_point->s() &&
             point.t() < bottom_left_point->t())) {
          bottom_left_point = point;
        }
      }
    }
  }
  return bottom_left_point;
}

void LateralTube::GetCorridorAxisLanesWidth(
    const CorridorPoint& corridor_point, const LocalRoute& cur_local_route,
    double& lanes_left_boundary, double& lanes_right_boundary) const {
  double left_lane_left_boundary = kDefaultLaneHalfWidth;
  double left_lane_right_boundary = kDefaultLaneHalfWidth;
  double right_lane_left_boundary = kDefaultLaneHalfWidth;
  double right_lane_right_boundary = kDefaultLaneHalfWidth;
  double curr_lane_left_boundary = kDefaultLaneHalfWidth;
  double curr_lane_right_boundary = kDefaultLaneHalfWidth;

  bool success = false;
  if (!cur_local_route.LeftRoute() && cur_local_route.RightRoute()) {
    success = cur_local_route.GetLaneWidth(corridor_point.s_ref,
                                           curr_lane_left_boundary,
                                           curr_lane_right_boundary);
    lanes_left_boundary = curr_lane_left_boundary;
    success = cur_local_route.RightRoute()->GetLaneWidth(
        corridor_point.s_ref, right_lane_left_boundary,
        right_lane_right_boundary);
    lanes_right_boundary = -right_lane_right_boundary -
                           (curr_lane_left_boundary + curr_lane_right_boundary);
  } else if (cur_local_route.LeftRoute() && !cur_local_route.RightRoute()) {
    success = cur_local_route.GetLaneWidth(corridor_point.s_ref,
                                           curr_lane_left_boundary,
                                           curr_lane_right_boundary);
    lanes_right_boundary = -curr_lane_right_boundary;
    success = cur_local_route.LeftRoute()->GetLaneWidth(
        corridor_point.s_ref, left_lane_left_boundary,
        left_lane_right_boundary);
    lanes_left_boundary = left_lane_left_boundary +
                          (curr_lane_left_boundary + curr_lane_right_boundary);
  } else if (cur_local_route.LeftRoute() && cur_local_route.RightRoute()) {
    success = cur_local_route.GetLaneWidth(corridor_point.s_ref,
                                           curr_lane_left_boundary,
                                           curr_lane_right_boundary);
    success = cur_local_route.LeftRoute()->GetLaneWidth(
        corridor_point.s_ref, left_lane_left_boundary,
        left_lane_right_boundary);
    lanes_left_boundary = left_lane_left_boundary +
                          (curr_lane_left_boundary + curr_lane_right_boundary);
    success = cur_local_route.RightRoute()->GetLaneWidth(
        corridor_point.s_ref, right_lane_left_boundary,
        right_lane_right_boundary);
    lanes_right_boundary = -right_lane_right_boundary -
                           (curr_lane_left_boundary + curr_lane_right_boundary);
  } else {
    success = cur_local_route.GetLaneWidth(corridor_point.s_ref,
                                           curr_lane_left_boundary,
                                           curr_lane_right_boundary);
    
    lanes_left_boundary = curr_lane_left_boundary;
    lanes_right_boundary = -curr_lane_right_boundary;
  }

  if (!success) {
    AERROR << "Failed to get lane width.";
  }
  lanes_left_boundary = lanes_left_boundary - corridor_point.l_ref;
  lanes_right_boundary = lanes_right_boundary - corridor_point.l_ref;
}

}  // namespace planning
}  // namespace zark