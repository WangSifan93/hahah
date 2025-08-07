#include "router/plan_passage_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "maps/map_def.h"
#include "maps/map_or_die_macros.h"
#include "maps/semantic_map_defs.h"
#include "maps/semantic_map_util.h"
#include "math/circle_fitter.h"
#include "math/geometry/segment2d.h"
#include "math/util.h"
#include "math/vec.h"
#include "plan/planner_defs.h"
#include "util/lane_path_util.h"
#include "util/lane_point_util.h"

namespace e2e_noa::planning {

namespace {

constexpr double kVehicleWidth = 2.02;
constexpr double kPlanPassageCutOffAngleDiff = 1.5 * M_PI;
constexpr double kPlanPassageMaxForwardExtendLength = 10.0;

constexpr double kFarStationHorizonRatio = 1.0 - 0.3;
constexpr double kFarRouteStationStep = 2.0 * kRouteStationUnitStep;

StationBoundaryType MapLaneBoundaryTypeToStationBoundaryType(
    ad_e2e::planning::LaneBoundaryType lane_type) {
  switch (lane_type.line_type) {
    case ad_e2e::planning::LineType::SOLID:
    case ad_e2e::planning::LineType::SOLID_SOLID:
    case ad_e2e::planning::LineType::SOLID_DASHED:
      switch (lane_type.line_color) {
        case ad_e2e::planning::LineColor::COLOR_YELLOW:
          return StationBoundaryType::SOLID_YELLOW;
        case ad_e2e::planning::LineColor::COLOR_WHITE:
        default:
          return StationBoundaryType::SOLID_WHITE;
      }
    case ad_e2e::planning::LineType::DASHED:
    case ad_e2e::planning::LineType::DASHED_DASHED:
    case ad_e2e::planning::LineType::DASHED_SOLID:
      switch (lane_type.line_color) {
        case ad_e2e::planning::LineColor::COLOR_YELLOW:
          return StationBoundaryType::BROKEN_YELLOW;
        case ad_e2e::planning::LineColor::COLOR_WHITE:
        default:
          return StationBoundaryType::BROKEN_WHITE;
      }
    case ad_e2e::planning::LineType::SHADED_AREA:
      return StationBoundaryType::SOLID_WHITE;
    case ad_e2e::planning::LineType::VIRTUAL_LANE:
    case ad_e2e::planning::LineType::VIRTUAL_JUNCTION:
      return StationBoundaryType::VIRTUAL_LANE;
    case ad_e2e::planning::LineType::UNKNOWN:
      return StationBoundaryType::UNKNOWN_TYPE;
    default:
      LOG(ERROR) << "Um-matched lane type: " << (int)lane_type.line_type;
      return StationBoundaryType::UNKNOWN_TYPE;
  }
}

bool LowerType(StationBoundaryType type1, StationBoundaryType type2) {
  switch (type1) {
    case StationBoundaryType::UNKNOWN_TYPE:
      return true;
    case StationBoundaryType::VIRTUAL_LANE:
      return StationBoundaryType::UNKNOWN_TYPE != type2 &&
             StationBoundaryType::VIRTUAL_LANE != type2;
    case StationBoundaryType::BROKEN_WHITE:
      return type2 == StationBoundaryType::SOLID_WHITE ||
             type2 == StationBoundaryType::BROKEN_YELLOW ||
             type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_WHITE:
    case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
    case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
      return type2 == StationBoundaryType::BROKEN_YELLOW ||
             type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::BROKEN_YELLOW:
      return type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_YELLOW:
      return type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      return type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::CURB:
    case StationBoundaryType::VIRTUAL_CURB:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

std::pair<double, double> ComputeLateralLimits(
    const std::vector<StationCenter>& centers,
    const StationCenter& current_center) {
  if (centers.empty()) return {-kMaxLateralOffset, kMaxLateralOffset};

  const double d_theta = NormalizeAngle(current_center.tangent.FastAngle() -
                                        centers.back().tangent.FastAngle());
  const double ds = current_center.accum_s - centers.back().accum_s;
  if (std::abs(d_theta) * kMaxLateralOffset < ds)
    return {-kMaxLateralOffset, kMaxLateralOffset};

  constexpr double kVehicleMinTurnRadius = 5.05;
  const double turn_radius =
      std::max(kVehicleMinTurnRadius, ds / std::abs(d_theta));

  if (d_theta < 0.0) {
    return {-turn_radius, kMaxLateralOffset};
  }

  return {-kMaxLateralOffset, turn_radius};
}

std::vector<StationBoundary> CollectStationBoundaries(
    const PlannerSemanticMapManager& psmm, const StationCenter& center,
    const double& max_right_offset, const double& max_left_offset,
    const absl::flat_hash_set<std::string>& lane_boundary_set,
    const bool ignore_curb, bool* has_cross_curb_ptr) {
  const Segment2d normal_line(center.lat_point(max_right_offset),
                              center.lat_point(max_left_offset));

  std::vector<StationBoundary> station_boundaries;

  const auto road_boundaries =
      psmm.GetRoadBoundaries(center.xy, kMaxLateralOffset);
  bool has_left_curb = false;
  bool has_right_curb = false;
  const double ignore_center_range_th = ad_e2e::planning::Constants::ZERO;
  for (const auto& boundary : road_boundaries) {
    if (ignore_curb) break;
    if (!boundary) continue;
    const auto& rb_type = boundary->type().boundary_type;
    if (rb_type == ad_e2e::planning::BoundaryType::OCC_VEGETATION ||
        rb_type == ad_e2e::planning::BoundaryType::VIRTUAL) {
      continue;
    }
    double most_left_offset = -1.0;
    double least_left_offset = 100.0;
    double most_right_offset = 1.0;
    double least_right_offset = -100.0;
    double closest_offset = std::numeric_limits<double>::max();
    for (int k = 0; k < boundary->curve_points().size(); ++k) {
      const Vec2d& cp = boundary->curve_points()[k];
      const Segment2d boundary_segment(center.xy, cp);
      if (normal_line.DistanceTo(cp) > 2.0 * kRouteStationUnitStep) continue;
      double proj_value = normal_line.ProjectOntoUnit(cp);
      if (proj_value < 0.0 || proj_value > max_left_offset - max_right_offset)
        continue;
      double lat_offset = proj_value + max_right_offset;
      if (lat_offset > 0) {
        most_left_offset = std::fmax(most_left_offset, lat_offset);
        least_left_offset = std::fmin(least_left_offset, lat_offset);
      } else {
        most_right_offset = std::fmin(most_right_offset, lat_offset);
        least_right_offset = std::fmax(least_right_offset, lat_offset);
      }
    }
    if (most_left_offset < -0.5) {
      if (least_right_offset > -50.0 &&
          least_right_offset < -ignore_center_range_th) {
        closest_offset = least_right_offset;
      }
    } else if (most_right_offset > 0.5) {
      if (least_left_offset < 50.0 &&
          least_left_offset > ignore_center_range_th) {
        closest_offset = least_left_offset;
      }
    } else {
      if (std::fmin(most_left_offset, -most_right_offset) >
              ad_e2e::planning::Constants::ZERO &&
          std::fabs(most_left_offset - most_right_offset) > 0.5 &&
          nullptr != has_cross_curb_ptr) {
        *has_cross_curb_ptr = true;
      }
    }
    if (closest_offset < std::numeric_limits<double>::max()) {
      if (boundary->id() == "2392468629959721216" ||
          boundary->id() == "2392468629959720961" ||
          boundary->id() == "2392468621369555456" ||
          boundary->id() == "2392468621369555200" ||
          boundary->id() == "2392468621369554944") {
        closest_offset += 0.35;
      }
      if (boundary->id() == "2392468629959704832") {
        closest_offset += 0.2;
      }
      if (boundary->id() == "2392468629959683328" ||
          boundary->id() == "2392468629959683584") {
        closest_offset += 0.65;
      }
      if (boundary->id() == "2392468629959638272") {
        closest_offset += 0.9;
      }
      station_boundaries.push_back({StationBoundaryType::CURB, closest_offset});
      closest_offset > 0 ? has_left_curb = true : has_right_curb = true;
    }
  }

  const auto clear_areas = psmm.GetClearAreas(center.xy, kMaxLateralOffset);
  for (const auto& clear_area : clear_areas) {
    if (ignore_curb) break;
    if (ad_e2e::planning::ImpassableAeraType::LINEAR_SAFE_ISLAND ==
        clear_area->type())
      continue;
    double min_sqr_dis = std::numeric_limits<double>::max();
    double closest_offset = 10.0;
    const double center_offset =
        normal_line.ProjectOntoUnit(clear_area->polygon().CircleCenter()) +
        max_right_offset;
    for (int k = 0; k < clear_area->points().size(); ++k) {
      const Vec2d& cp = clear_area->points()[k];
      const Segment2d boundary_segment(center.xy, cp);
      if (normal_line.DistanceTo(cp) > 1.1 * kRouteStationUnitStep) continue;
      double proj_value = normal_line.ProjectOntoUnit(cp);
      if (proj_value < 0.0 || proj_value > max_left_offset - max_right_offset)
        continue;
      double lat_offset = proj_value + max_right_offset;
      if (lat_offset * center_offset < 0.0) continue;
      double sqr_dis = std::fabs(lat_offset);
      if (sqr_dis < min_sqr_dis) {
        min_sqr_dis = sqr_dis;
        closest_offset = lat_offset;
      }
    }
    if (min_sqr_dis < std::numeric_limits<double>::max()) {
      station_boundaries.push_back({StationBoundaryType::CURB, closest_offset});
      closest_offset > 0 ? has_left_curb = true : has_right_curb = true;
    }
  }

  if (!has_left_curb)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_CURB, max_left_offset});
  if (!has_right_curb)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_CURB, max_right_offset});

  bool has_left_lane_boundary = false;
  bool has_right_lane_boundary = false;
  const auto lane_boundaries =
      psmm.GetLaneBoundaries(center.xy, kMaxLateralOffset);
  const bool is_merge_or_split = center.is_merging || center.is_splitting;
  for (const auto& boundary : lane_boundaries) {
    if (!lane_boundary_set.empty() &&
        !lane_boundary_set.contains(boundary->id())) {
      continue;
    }
    Vec2d closest_intersection;
    double min_sqr_dis = std::numeric_limits<double>::max();
    for (int k = 0; k + 1 < boundary->points().size(); ++k) {
      const Vec2d& p0 = boundary->points()[k];
      const Vec2d& p1 = boundary->points()[k + 1];
      const Segment2d boundary_segment(p0, p1);
      Vec2d intersection;
      if (!normal_line.GetIntersect(boundary_segment, &intersection)) continue;
      const double sqr_dis = center.xy.DistanceSquareTo(intersection);
      if (sqr_dis < min_sqr_dis) {
        min_sqr_dis = sqr_dis;
        closest_intersection = intersection;
      }
    }
    if (min_sqr_dis < std::numeric_limits<double>::max()) {
      auto station_boundary =
          MapLaneBoundaryTypeToStationBoundaryType(boundary->type());
      if (!is_merge_or_split ||
          (StationBoundaryType::SOLID_YELLOW == station_boundary ||
           StationBoundaryType::SOLID_WHITE == station_boundary)) {
        const auto lat_offset = center.lat_offset(closest_intersection);
        if (StationBoundaryType::UNKNOWN_TYPE != station_boundary) {
          station_boundaries.push_back({station_boundary, lat_offset});
          lat_offset > 0 ? has_left_lane_boundary = true
                         : has_right_lane_boundary = true;
        } else {
          const auto abs_lat_offset = std::fabs(lat_offset);
          if (abs_lat_offset > kMinHalfLaneWidth - 0.3 &&
              abs_lat_offset < kMaxHalfLaneWidth) {
            station_boundaries.push_back(
                {StationBoundaryType::VIRTUAL_LANE, lat_offset});
            lat_offset > 0 ? has_left_lane_boundary = true
                           : has_right_lane_boundary = true;
          }
        }
      }
    }
  }
  if (!has_left_lane_boundary)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_LANE, kDefaultHalfLaneWidth});
  if (!has_right_lane_boundary)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_LANE, -kDefaultHalfLaneWidth});

  return station_boundaries;
}

void PostProcessStationBoundaries(
    const StationCenter& center,
    std::vector<StationBoundary>* mutable_boundaries) {
  std::stable_sort(mutable_boundaries->begin(), mutable_boundaries->end(),
                   [](const StationBoundary& lhs, const StationBoundary& rhs) {
                     return lhs.lat_offset < rhs.lat_offset;
                   });
  std::vector<StationBoundary> remaining_boundaries;
  const double min_half_virtual_lane_width = 0.9;

  auto current_top_type = StationBoundaryType::UNKNOWN_TYPE;
  for (auto it = mutable_boundaries->rbegin(); it != mutable_boundaries->rend();
       ++it) {
    if (it->lat_offset > 0.0 || LowerType(it->type, current_top_type)) {
      continue;
    }
    if (StationBoundaryType::VIRTUAL_LANE == it->type &&
        it->lat_offset > -min_half_virtual_lane_width) {
      continue;
    }

    remaining_boundaries.push_back(*it);
    if (it->type == StationBoundaryType::CURB ||
        it->type == StationBoundaryType::VIRTUAL_CURB) {
      break;
    }
    current_top_type = it->type;
  }
  std::reverse(remaining_boundaries.begin(), remaining_boundaries.end());

  current_top_type = StationBoundaryType::UNKNOWN_TYPE;
  for (const auto& bound : *mutable_boundaries) {
    if (bound.lat_offset < 0.0 || LowerType(bound.type, current_top_type)) {
      continue;
    }
    if (StationBoundaryType::VIRTUAL_LANE == bound.type &&
        bound.lat_offset < min_half_virtual_lane_width) {
      continue;
    }

    remaining_boundaries.push_back(bound);
    if (bound.type == StationBoundaryType::CURB ||
        bound.type == StationBoundaryType::VIRTUAL_CURB) {
      break;
    }
    current_top_type = bound.type;
  }

  const double borrow_road_width = kDefaultLaneWidth + kVehicleWidth * 0.5;
  double left_solid_yellow_line_offset = std::numeric_limits<double>::max();
  double right_solid_yellow_line_offset = std::numeric_limits<double>::lowest();
  for (const auto& bound : remaining_boundaries) {
    if (bound.type == StationBoundaryType::SOLID_YELLOW ||
        bound.type == StationBoundaryType::SOLID_DOUBLE_YELLOW) {
      bound.lat_offset < 0.0 ? right_solid_yellow_line_offset = bound.lat_offset
                             : left_solid_yellow_line_offset = bound.lat_offset;
    }
  }
  if (left_solid_yellow_line_offset + borrow_road_width <
      remaining_boundaries.back().lat_offset) {
    remaining_boundaries.back().type = StationBoundaryType::VIRTUAL_CURB;
    remaining_boundaries.back().lat_offset =
        left_solid_yellow_line_offset + borrow_road_width;
  }
  if (right_solid_yellow_line_offset - borrow_road_width >
      remaining_boundaries.front().lat_offset) {
    remaining_boundaries.front().type = StationBoundaryType::VIRTUAL_CURB;
    remaining_boundaries.front().lat_offset =
        right_solid_yellow_line_offset - borrow_road_width;
  }

  *mutable_boundaries = std::move(remaining_boundaries);
}

struct PlanPassageData {
  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
};

PlanPassageData SampleLanePathWithPlannerSemanticMapMgr(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double start_s, double end_s, double step, bool avoid_loop,
    std::optional<double> override_speed_limit_mps) {
  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
  const int n = CeilToInteger((end_s - start_s) / step);
  centers.reserve(n);
  stations_boundaries.reserve(n);

  int station_idx = 0;
  double prev_station_angle = 0.0;
  double max_accumulated_angle_diff = 0.0;
  double min_accumulated_angle_diff = 0.0;

  for (double sample_s = start_s; sample_s <= end_s; sample_s += step) {
    const auto sample_lane_point = lane_path.ArclengthToLanePoint(sample_s);

    if (station_idx == 0) {
      prev_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
    } else {
      const auto cur_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
      const double angle_diff =
          NormalizeAngle(cur_station_angle - prev_station_angle);
      max_accumulated_angle_diff =
          std::max(max_accumulated_angle_diff + angle_diff, angle_diff);
      min_accumulated_angle_diff =
          std::min(min_accumulated_angle_diff + angle_diff, angle_diff);
      prev_station_angle = cur_station_angle;
    }
    if ((max_accumulated_angle_diff >= kPlanPassageCutOffAngleDiff ||
         min_accumulated_angle_diff <= -kPlanPassageCutOffAngleDiff) &&
        avoid_loop) {
      break;
    }
    station_idx++;

    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm,
                                      sample_lane_point.lane_id());

    StationCenter center{
        .lane_id = sample_lane_point.lane_id(),
        .fraction = sample_lane_point.fraction(),
        .xy = ComputeLanePointPos(psmm, sample_lane_point),
        .tangent = ComputeLanePointTangent(psmm, sample_lane_point),
        .accum_s = sample_s,
        .speed_limit =
            override_speed_limit_mps.has_value()
                ? *override_speed_limit_mps
                : psmm.QueryLaneSpeedLimitById(sample_lane_point.lane_id()),
        .is_virtual = lane_info.IsVirtual(),
        .is_merging = lane_info.is_merge(),
        .is_splitting = lane_info.is_split(),
        .is_in_intersection = !lane_info.junction_id().empty(),
        .has_cross_curb = false,
        .turn_type = lane_info.turn_type()};

    absl::flat_hash_set<std::string> lane_boundaries;
    lane_boundaries.reserve(
        lane_info.left_boundary()->lane_boundaries().size() +
        lane_info.right_boundary()->lane_boundaries().size());
    lane_boundaries.clear();
    for (const auto& bound : lane_info.left_boundary()->lane_boundaries()) {
      lane_boundaries.insert(bound->id());
    }
    for (const auto& bound : lane_info.right_boundary()->lane_boundaries()) {
      lane_boundaries.insert(bound->id());
    }

    const auto [right_lat_offset, left_lat_offset] =
        ComputeLateralLimits(centers, center);
    auto current_station_boundaries = CollectStationBoundaries(
        psmm, center, right_lat_offset, left_lat_offset, lane_boundaries, false,
        &center.has_cross_curb);
    PostProcessStationBoundaries(center, &current_station_boundaries);

    centers.emplace_back(std::move(center));
    stations_boundaries.emplace_back(std::move(current_station_boundaries));
  }
  return PlanPassageData{.centers = std::move(centers),
                         .stations_boundaries = std::move(stations_boundaries)};
}

void ExtendPlanPassageDataByCircleFit(double fit_length, double step_s,
                                      PlanPassageData* dp_data_ptr) {
  constexpr int kFitDataMaxSize = 30;
  constexpr int kFitEveryNPt = 1;

  auto& centers = dp_data_ptr->centers;
  auto& stations_boundaries = dp_data_ptr->stations_boundaries;

  std::vector<Vec2d> fit_data;
  std::vector<double> weights;
  fit_data.reserve(kFitDataMaxSize);
  weights.reserve(kFitDataMaxSize);
  for (int i = centers.size() - 1; i >= 0; i -= kFitEveryNPt) {
    fit_data.push_back(centers[i].xy);
    weights.push_back(Sqr(kFitDataMaxSize - fit_data.size()));
    if (fit_data.size() >= kFitDataMaxSize) break;
  }
  double mse = 0.0;
  const auto circle_or = FitCircleToData(fit_data, weights, SVD, &mse);
  if (!circle_or.ok()) {
    LOG(WARNING) << "Plan passage forward fitting failed: "
                 << circle_or.status().message();
    return;
  }
  constexpr double kMaxFittingMSError = 1e-3;
  if (mse > kMaxFittingMSError) {
    LOG(WARNING) << "Plan passage forward fitting failed: fitting error " << mse
                 << " is too large.";
    return;
  }
  const auto& circle = *circle_or;
  constexpr double kMinFittedRadius = 60.0;
  constexpr double kMaxFittedRadius = 2000.0;
  if (circle.radius() < kMinFittedRadius ||
      circle.radius() > kMaxFittedRadius) {
    LOG(WARNING) << "Plan passage forward fitting failed: fitted radius "
                 << circle.radius() << " out of reasonable range ("
                 << kMinFittedRadius << ", " << kMaxFittedRadius << ").";
    return;
  }

  double last_accum_s = centers.back().accum_s;
  const double d_theta = std::copysign(
      step_s / circle.radius(),
      AngleDifference(circle.EvaluateTheta(centers[centers.size() - 2].xy),
                      circle.EvaluateTheta(centers.back().xy)));
  while (last_accum_s < fit_length) {
    auto center = centers.back();
    center.lane_id = mapping::kInvalidElementId;
    center.fraction = 1.0;
    center.is_virtual = true;
    center.is_splitting = false;

    const Vec2d new_pos =
        circle.EvaluateXY(circle.EvaluateTheta(center.xy) + d_theta);
    center.tangent = (new_pos - center.xy).normalized();
    last_accum_s += center.xy.DistanceTo(new_pos);
    center.xy = new_pos;
    center.accum_s = last_accum_s;

    centers.emplace_back(std::move(center));
    stations_boundaries.push_back(
        {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
         {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
  }
}

}  

absl::StatusOr<PlanPassage> BuildPlanPassageFromLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step_s, bool avoid_loop, double backward_extend_len,
    double required_planning_horizon,
    std::optional<double> override_speed_limit_mps, FrenetFrameType type) {
  double start_s;
  mapping::LanePath extended_lane_path;
  if (backward_extend_len > 0.0) {
    extended_lane_path =
        BackwardExtendLanePath(psmm, lane_path, backward_extend_len);
    start_s = extended_lane_path.FirstOccurrenceOfLanePointToArclength(
        lane_path.front());
  } else {
    start_s = 0.0;
    extended_lane_path = lane_path;
  }

  auto dp_data = SampleLanePathWithPlannerSemanticMapMgr(
      psmm, extended_lane_path, 0.0, extended_lane_path.length(), step_s,
      avoid_loop, override_speed_limit_mps);

  const double forward_extend_len =
      required_planning_horizon - lane_path.length();
  const auto forward_extend_lane_path =
      ForwardExtendLanePath(psmm, lane_path, forward_extend_len, false);
  if (forward_extend_len > 0.0 &&
      forward_extend_lane_path.back() == lane_path.back()) {
    const int n = CeilToInteger(required_planning_horizon / step_s) + 1;
    dp_data.centers.reserve(n);
    dp_data.stations_boundaries.reserve(n);

    double last_accum_s = dp_data.centers.back().accum_s;
    constexpr double kMaxFittedLength = 100.0;
    constexpr double kMinLaneLengthForFitting = 40.0;
    const double fit_length =
        std::min(required_planning_horizon, kMaxFittedLength);
    if (last_accum_s < fit_length && last_accum_s >= kMinLaneLengthForFitting) {
      ExtendPlanPassageDataByCircleFit(fit_length, step_s, &dp_data);
    }

    last_accum_s = dp_data.centers.back().accum_s;
    while ((last_accum_s += step_s) <= required_planning_horizon) {
      auto center = dp_data.centers.back();
      center.lane_id = mapping::kInvalidElementId;
      center.fraction = 1.0;
      center.xy += center.tangent * step_s;
      center.accum_s = last_accum_s;
      center.is_virtual = true;
      center.is_splitting = false;

      dp_data.centers.emplace_back(std::move(center));
      dp_data.stations_boundaries.push_back(
          {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
           {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
    }
  }
  if (dp_data.centers.size() < 2) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Too few stations in BuildPlanPassageFromLanePath: Lane path: ",
        lane_path.DebugString(),
        "\nExtended lane path: ", extended_lane_path.DebugString()));
  }

  StationVector<Station> stations;
  stations.reserve(dp_data.centers.size());
  for (int i = 0; i < dp_data.centers.size(); ++i) {
    if (backward_extend_len > 0.0) {
      dp_data.centers[i].accum_s -= start_s;
    }
    stations.emplace_back(std::move(dp_data.centers[i]),
                          std::move(dp_data.stations_boundaries[i]));
  }

  return PlanPassage(std::move(stations), lane_path,
                     std::move(extended_lane_path), start_s, false, type, {});
}

bool IsChangeStation(StationInfo last, StationInfo cur) {
  if ((last.turn_type != cur.turn_type) ||
      (last.split_topo != cur.split_topo) ||
      (last.lane_type != LaneType::LANE_ROUND_ABOUT &&
       cur.lane_type == LaneType::LANE_ROUND_ABOUT) ||
      (last.lane_type == LaneType::LANE_ROUND_ABOUT &&
       cur.lane_type != LaneType::LANE_ROUND_ABOUT)) {
    return true;
  }
  return false;
}

absl::StatusOr<PlanPassage> BuildPlanPassage(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& lane_path_from_pose,
    const mapping::LanePath& backward_extended_lane_path,
    const mapping::LanePoint& anchor_point, double planning_horizon,
    const mapping::LanePoint& destination, bool all_lanes_virtual,
    std::optional<double> override_speed_limit_mps, FrenetFrameType type,
    ad_e2e::planning::LaneSeqInfoPtr lane_seq_info) {
  auto lane_path_in_horizon =
      lane_path_from_pose.BeforeArclength(planning_horizon);

  const double forward_extend_len =
      std::min(planning_horizon - lane_path_from_pose.length(),
               kPlanPassageMaxForwardExtendLength);
  auto ref_lane_path = ForwardExtendLanePath(
      psmm,
      backward_extended_lane_path.BeforeFirstOccurrenceOfLanePoint(
          lane_path_in_horizon.back()),
      forward_extend_len, false);

  const double ref_ego_s = ref_lane_path.FirstOccurrenceOfLanePointToArclength(
      lane_path_from_pose.front());
  const double ref_anchor_s =
      ref_lane_path.ContainsLanePoint(anchor_point)
          ? ref_lane_path.FirstOccurrenceOfLanePointToArclength(anchor_point)
          : ref_ego_s;
  const double ref_neutral_s =
      ref_anchor_s +
      RoundToInt((ref_ego_s - ref_anchor_s) / kRouteStationUnitStep) *
          kRouteStationUnitStep;
  const double start_station_accum_s =
      -FloorToInt(ref_neutral_s / kRouteStationUnitStep) *
      kRouteStationUnitStep;
  const double start_station_ref_s = ref_neutral_s + start_station_accum_s;

  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
  const int n = CeilToInteger((planning_horizon + ref_neutral_s) /
                              kRouteStationUnitStep) +
                1;
  centers.reserve(n);
  stations_boundaries.reserve(n);

  int station_idx = 0;
  double prev_station_angle = 0.0;
  double max_accumulated_angle_diff = 0.0;
  double min_accumulated_angle_diff = 0.0;

  const double far_station_thres =
      kFarStationHorizonRatio * planning_horizon + ref_ego_s;
  double sample_s = start_station_ref_s;
  const double loaded_length = ref_lane_path.length();
  bool angle_diff_cutoff = false;
  e2e_noa::mapping::LanePoint last_point;
  StationInfo last_station_info{};
  std::vector<int> change_index;
  while (sample_s <= loaded_length) {
    const auto sample_lane_point = ref_lane_path.ArclengthToLanePoint(sample_s);

    if (station_idx == 0) {
      prev_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
    } else {
      const auto cur_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
      const double angle_diff =
          NormalizeAngle(cur_station_angle - prev_station_angle);
      max_accumulated_angle_diff =
          std::max(max_accumulated_angle_diff + angle_diff, angle_diff);
      min_accumulated_angle_diff =
          std::min(min_accumulated_angle_diff + angle_diff, angle_diff);
      prev_station_angle = cur_station_angle;
    }
    if (max_accumulated_angle_diff >= kPlanPassageCutOffAngleDiff ||
        min_accumulated_angle_diff <= -kPlanPassageCutOffAngleDiff) {
      angle_diff_cutoff = true;
      break;
    }
    if (station_idx > 0) {
      auto now_pt = ComputeLanePointPos(psmm, sample_lane_point);
      auto last_pt = ComputeLanePointPos(psmm, last_point);
      auto two_pt_dis =
          std::hypot((now_pt.x() - last_pt.x()), (now_pt.y() - last_pt.y()));
      if (two_pt_dis > 5.0) break;
    }
    last_point = sample_lane_point;

    station_idx++;

    SMM_ASSIGN_LANE_OR_BREAK_ISSUE(lane_info, psmm,
                                   sample_lane_point.lane_id());

    double speed_limit =
        override_speed_limit_mps.has_value()
            ? *override_speed_limit_mps
            : psmm.QueryLaneSpeedLimitById(sample_lane_point.lane_id());

    StationInfo station_info{
        .is_in_intersection = !lane_info.junction_id().empty(),
        .is_exclusive_right_turn =
            (lane_info.turn_type() == ad_e2e::planning::RIGHT_TURN),
        .is_in_roundabout =
            (lane_info.type() == ad_e2e::planning::LANE_ROUND_ABOUT),
        .speed_limit = speed_limit,
        .lane_type = lane_info.type(),
        .turn_type = lane_info.turn_type(),
        .split_topo = lane_info.split_topology(),
        .merge_topo = lane_info.merge_topology()};

    if (!centers.empty() && IsChangeStation(last_station_info, station_info)) {
      change_index.emplace_back(centers.size());
    }
    last_station_info = station_info;

    StationCenter center{
        .lane_id = sample_lane_point.lane_id(),
        .fraction = sample_lane_point.fraction(),
        .xy = ComputeLanePointPos(psmm, sample_lane_point),
        .tangent = ComputeLanePointTangent(psmm, sample_lane_point),
        .accum_s = sample_s - ref_neutral_s,
        .speed_limit = speed_limit,
        .is_virtual = (all_lanes_virtual || lane_info.IsVirtual()),
        .is_merging = lane_info.is_merge(),
        .is_splitting = lane_info.is_split(),
        .is_in_intersection = !lane_info.junction_id().empty(),
        .has_cross_curb = false,
        .turn_type = lane_info.turn_type(),
        .station_info = std::move(station_info)};

    absl::flat_hash_set<std::string> lane_boundaries;
    lane_boundaries.reserve(
        lane_info.left_boundary()->lane_boundaries().size() +
        lane_info.right_boundary()->lane_boundaries().size());
    lane_boundaries.clear();
    for (const auto& bound : lane_info.left_boundary()->lane_boundaries()) {
      if ((e2e_noa::mapping::IsRightMostLane((*psmm.map_ptr()),
                                             lane_info.id()) &&
           (!e2e_noa::mapping::IsLeftMostLane((*psmm.map_ptr()),
                                              lane_info.id()))) &&
          lane_info.is_split()) {
        continue;
      }
      lane_boundaries.insert(bound->id());
    }
    for (const auto& bound : lane_info.right_boundary()->lane_boundaries()) {
      if ((e2e_noa::mapping::IsLeftMostLane((*psmm.map_ptr()),
                                            lane_info.id()) &&
           (!e2e_noa::mapping::IsRightMostLane((*psmm.map_ptr()),
                                               lane_info.id()))) &&
          lane_info.is_split()) {
        continue;
      }
      lane_boundaries.insert(bound->id());
    }

    const auto [right_lat_offset, left_lat_offset] =
        ComputeLateralLimits(centers, center);
    std::vector<StationBoundary> current_station_boundaries;

    current_station_boundaries = CollectStationBoundaries(
        psmm, center, right_lat_offset, left_lat_offset, lane_boundaries,
        sample_s < ref_ego_s - 2.5, &center.has_cross_curb);
    PostProcessStationBoundaries(center, &current_station_boundaries);

    centers.push_back(std::move(center));
    stations_boundaries.push_back(std::move(current_station_boundaries));

    sample_s += sample_s >= far_station_thres ? kFarRouteStationStep
                                              : kRouteStationUnitStep;
  }

  if (!angle_diff_cutoff &&
      lane_path_in_horizon.back() == ref_lane_path.back()) {
    double last_accum_s = centers.back().accum_s;
    while ((last_accum_s += kFarRouteStationStep) <= planning_horizon) {
      auto center = centers.back();
      center.lane_id = mapping::kInvalidElementId;
      center.fraction = 1.0;
      center.xy += center.tangent * kFarRouteStationStep;
      center.accum_s = last_accum_s;
      center.is_virtual = true;
      center.is_splitting = false;
      center.turn_type = ad_e2e::planning::NO_TURN;

      centers.push_back(std::move(center));
      stations_boundaries.push_back(
          {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
           {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
    }
  }
  if (centers.size() < 2) {
    std::cout << "Too few stations in BuildPlanPassage: Lane path: "
              << lane_path_from_pose.DebugString()
              << "\nExtended lane path: " << ref_lane_path.DebugString()
              << std::endl;
    return absl::FailedPreconditionError(
        absl::StrCat("Too few stations in BuildPlanPassage: Lane path: ",
                     lane_path_from_pose.DebugString(),
                     "\nExtended lane path: ", ref_lane_path.DebugString()));
  }

  StationVector<Station> stations;
  stations.reserve(centers.size());
  for (int i = 0; i < centers.size(); ++i) {
    stations.emplace_back(std::move(centers[i]),
                          std::move(stations_boundaries[i]));
  }

  const bool reach_destination =
      lane_path_in_horizon.ContainsLanePoint(destination);
  return PlanPassage(std::move(stations), std::move(lane_path_in_horizon),
                     std::move(ref_lane_path), ref_ego_s - ref_neutral_s,
                     reach_destination, type, std::move(change_index),
                     lane_seq_info);
}

}  
