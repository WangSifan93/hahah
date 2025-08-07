#include "util/spatial_search_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "gflags/gflags.h"
#include "maps/map_or_die_macros.h"
#include "maps/maps_helper.h"
#include "maps/semantic_map_util.h"
#include "math/util.h"
#include "util/lane_point_util.h"

DEFINE_bool(planner_enable_semantic_map_spatial_search, true,
            "switch to choose get map element method.");

namespace e2e_noa::planning {
namespace {

bool FindClosestFractionOnLaneSegment(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    mapping::ElementId lane_id, double heading, double* fraction,
    double heading_penalty_weight, Vec2d* closest_point_on_lane,
    double start_fraction, double end_fraction) {
  CHECK_GE(start_fraction, 0.0);
  CHECK_LE(start_fraction, 1.0);
  CHECK_GE(end_fraction, 0.0);
  CHECK_LE(end_fraction, 1.0);
  if (end_fraction - start_fraction < mapping::kEpsilon) {
    *fraction = start_fraction;
    if (closest_point_on_lane != nullptr) {
      *closest_point_on_lane = ComputeLanePointPos(
          psmm, mapping::LanePoint(lane_id, start_fraction));
    }
    return true;
  }
  const auto& lane_ptr = psmm.FindCurveLaneByIdOrNull(lane_id);
  if (lane_ptr == nullptr || lane_ptr->center_line().points().empty()) {
    return false;
  }
  const ad_e2e::planning::Lane& lane = *lane_ptr;
  const bool resampled = start_fraction > 0.0 || end_fraction < 1.0;
  std::vector<double> resampled_cumulative_lengths;
  const auto& points = resampled ? mapping::ResampleLanePoints(
                                       lane, start_fraction, end_fraction,
                                       &resampled_cumulative_lengths)
                                 : lane.points();

  const auto& cumulative_lengths = resampled
                                       ? resampled_cumulative_lengths
                                       : lane.center_line().GetAccuLength();
  double min_sqr_error = std::numeric_limits<double>::infinity();
  double min_d_cumulative_length{0};
  for (int j = 0; j + 1 < points.size(); ++j) {
    const Vec2d p0(points[j]);
    const Vec2d p1(points[j + 1]);

    const Vec2d segment = p1 - p0;
    const double l_sqr = segment.squaredNorm();
    if (l_sqr < mapping::kEpsilon) {
      LOG(WARNING) << absl::StrFormat("Adjacent points same in lane %d!!!",
                                      lane_id);
      continue;
    }
    const Vec2d query_segment = query_point - p0;
    const double projection = query_segment.dot(segment) / l_sqr;
    double d_sqr = 0.0;
    double cumulative_length = 0.0;
    Vec2d closest_point;
    if (projection < 0.0) {
      d_sqr = p0.DistanceSquareTo(query_point);
      closest_point = p0;
      cumulative_length = cumulative_lengths[j];
    } else if (projection > 1.0) {
      d_sqr = p1.DistanceSquareTo(query_point);
      closest_point = p1;
      cumulative_length = cumulative_lengths[j + 1];
    } else {
      d_sqr = Sqr(query_segment.CrossProd(segment.normalized()));
      closest_point = Lerp(p0, p1, projection);
      cumulative_length =
          Lerp(cumulative_lengths[j], cumulative_lengths[j + 1], projection);
    }

    const double error =
        d_sqr + heading_penalty_weight *
                    Sqr(NormalizeAngle(segment.FastAngle() - heading));
    if (min_sqr_error > error) {
      min_sqr_error = error;
      min_d_cumulative_length = cumulative_length;
      if (closest_point_on_lane != nullptr) {
        *closest_point_on_lane = closest_point;
      }
    }
  }
  const double length = lane.curve_length();
  if (length < mapping::kEpsilon) {
    return false;
  }
  *fraction = std::clamp(min_d_cumulative_length / length, start_fraction,
                         end_fraction);
  return true;
}

template <typename T>
absl::StatusOr<mapping::LanePoint> FindClosestBySpatialSearchAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    const T& lane_ids, double heading, double heading_penalty_weight,
    Vec2d* closest_point_on_lane, double start_fraction, double end_fraction,
    double cutoff_distance) {
  double fraction = 0.0;
  Vec2d closest_point;
  double lane_dist = std::numeric_limits<double>::infinity();
  double min_dist = std::numeric_limits<double>::infinity();
  int lane_size = lane_ids.size();
  bool success = false;
  if (lane_size > 0) {
    if (lane_size == 1) {
      if (!FindClosestFractionOnLaneSegment(
              psmm, query_point, lane_ids[0], heading, &fraction,
              heading_penalty_weight, closest_point_on_lane, start_fraction,
              end_fraction)) {
        return absl::NotFoundError(
            "Could not find nearest point by given lane id.");
      }
      return mapping::LanePoint(lane_ids[0], fraction);
    }
    std::optional<mapping::LanePoint> nearest_point;
    for (int i = 0; i < lane_size; ++i) {
      success = psmm.GetLaneProjection(query_point, lane_ids[i], &fraction,
                                       &closest_point, &lane_dist);
      if (!success) continue;
      mapping::LanePoint lane_point(lane_ids[i], fraction);
      const double d_sqr =
          lane_dist * lane_dist +
          heading_penalty_weight *
              Sqr(NormalizeAngle(
                  NormalizeAngle2D(ComputeLanePointTangent(psmm, lane_point)) -
                  heading));
      if (d_sqr < min_dist) {
        min_dist = d_sqr;
        if (closest_point_on_lane != nullptr) {
          *closest_point_on_lane = closest_point;
        }
        if (i == 0 && fraction < start_fraction) {
          if (!FindClosestFractionOnLaneSegment(
                  psmm, query_point, lane_ids[i], heading, &fraction,
                  heading_penalty_weight, closest_point_on_lane, start_fraction,
                  1.0)) {
            continue;
          }
        }
        if (i == lane_size - 1 && fraction > end_fraction) {
          if (!FindClosestFractionOnLaneSegment(
                  psmm, query_point, lane_ids[i], heading, &fraction,
                  heading_penalty_weight, closest_point_on_lane, 0.0,
                  end_fraction)) {
            continue;
          }
        }
        nearest_point = mapping::LanePoint(lane_ids[i], fraction);
      }
    }
    if (nearest_point.has_value()) {
      return *nearest_point;
    } else {
      return absl::NotFoundError(
          "Could not find nearest lane in provided lanes.");
    }
  }
  if (cutoff_distance > 10.0 * kSearchRadiusThreshold) {
    LOG(WARNING) << absl::StrFormat(
        "cutoff_distance %f may be too large for efficiency", cutoff_distance);
  }
  auto lane_ptrs = psmm.GetLanesInRadius(query_point, cutoff_distance);
  std::optional<mapping::LanePoint> nearest_point;
  for (const auto& lane_ptr : lane_ptrs) {
    if (lane_ptr == nullptr) continue;
    const auto& lane_id = lane_ptr->id();
    success = psmm.GetLaneProjection(query_point, lane_id, &fraction,
                                     &closest_point, &lane_dist);
    if (!success) continue;
    fraction = std::clamp(fraction, 0.0, 1.0);
    const mapping::LanePoint lane_point(lane_id, fraction);
    const double d_sqr =
        lane_dist * lane_dist +
        heading_penalty_weight *
            Sqr(NormalizeAngle(
                NormalizeAngle2D(ComputeLanePointTangent(psmm, lane_point)) -
                heading));
    if (d_sqr < min_dist) {
      min_dist = d_sqr;
      if (closest_point_on_lane != nullptr) {
        *closest_point_on_lane = closest_point;
      }
      nearest_point = lane_point;
    }
  }
  if (nearest_point.has_value()) {
    return *nearest_point;
  } else {
    return absl::NotFoundError(absl::StrCat(
        "Could not find nearest lane within distance: ", cutoff_distance));
  }
}

std::optional<mapping::LanePoint> FindClosestOnLaneBySpatialSearchAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    mapping::ElementId lane_id, double heading, double heading_penalty_weight,
    Vec2d* closest_point_on_lane, double start_fraction, double end_fraction,
    double cutoff_distance) {
  double fraction = 0.0;
  double lane_dist = std::numeric_limits<double>::infinity();
  if (!psmm.GetLaneProjection(query_point, lane_id, &fraction,
                              closest_point_on_lane, &lane_dist)) {
    return std::nullopt;
  }
  if (lane_dist > cutoff_distance) {
    return std::nullopt;
  }
  if (fraction < start_fraction || fraction > end_fraction) {
    if (!FindClosestFractionOnLaneSegment(psmm, query_point, lane_id, heading,
                                          &fraction, heading_penalty_weight,
                                          closest_point_on_lane, start_fraction,
                                          end_fraction)) {
      return std::nullopt;
    }
  }
  return mapping::LanePoint(lane_id, fraction);
}

}  // namespace

absl::StatusOr<mapping::LanePoint> FindClosestLanePointToSmoothPointAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    Vec2d* closest_point_on_lane, double cutoff_distance) {
  return FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel(
      psmm, query_point, 0.0, 0.0, closest_point_on_lane, cutoff_distance);
}

absl::StatusOr<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    double heading, double heading_penalty_weight, Vec2d* closest_point_on_lane,
    double cutoff_distance) {
  return FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
      psmm, query_point, std::vector<mapping::ElementId>(), heading,
      heading_penalty_weight, closest_point_on_lane, 0.0, 1.0, cutoff_distance);
}

absl::StatusOr<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    const mapping::LanePath& lane_path, double heading,
    double heading_penalty_weight, Vec2d* closest_point_on_lane,
    double cutoff_distance) {
  return FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
      psmm, query_point, lane_path.lane_ids(), heading, heading_penalty_weight,
      closest_point_on_lane, lane_path.start_fraction(),
      lane_path.end_fraction(), cutoff_distance);
}

std::optional<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    mapping::ElementId lane_id, double heading, double heading_penalty_weight,
    Vec2d* closest_point_on_lane, double start_fraction, double end_fraction,
    double cutoff_distance) {
  if (FLAGS_planner_enable_semantic_map_spatial_search) {
    return FindClosestOnLaneBySpatialSearchAtLevel(
        psmm, query_point, lane_id, heading, heading_penalty_weight,
        closest_point_on_lane, start_fraction, end_fraction, cutoff_distance);
  }
  double fraction = 0.0;
  if (!FindClosestFractionOnLaneSegment(psmm, query_point, lane_id, heading,
                                        &fraction, heading_penalty_weight,
                                        closest_point_on_lane, start_fraction,
                                        end_fraction)) {
    return std::nullopt;
  }
  return mapping::LanePoint(lane_id, fraction);
}

template <typename T>
absl::StatusOr<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    const T& lane_ids, double heading, double heading_penalty_weight,
    Vec2d* closest_point_on_lane, double start_fraction, double end_fraction,
    double cutoff_distance) {
  if (FLAGS_planner_enable_semantic_map_spatial_search) {
    return FindClosestBySpatialSearchAtLevel(
        psmm, query_point, lane_ids, heading, heading_penalty_weight,
        closest_point_on_lane, start_fraction, end_fraction, cutoff_distance);
  }
  const auto& all_lane_info = psmm.map_ptr()->lanes();
  double min_d_sqr = std::numeric_limits<double>::infinity();
  std::optional<mapping::ElementId> min_d_lane_id;
  double min_d_cumulative_length = 0.0;
  Vec2d min_d_closest_point_on_lane;

  for (int i = 0;
       i < (lane_ids.empty() ? all_lane_info.size() : lane_ids.size()); ++i) {
    const mapping::ElementId& lane_id =
        lane_ids.empty() ? all_lane_info[i]->id() : lane_ids[i];
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, lane_id);

    if (!lane_info.points().size() ||
        (cutoff_distance < std::numeric_limits<double>::infinity() &&
         std::max((query_point - lane_info.points()[0]).squaredNorm(),
                  (query_point - lane_info.points().back()).squaredNorm()) >
             Sqr(cutoff_distance + mapping::kEpsilon +
                 lane_info.curve_length()))) {
      continue;
    }

    double resample_start_fraction = 0.0;
    if (start_fraction > 0.0 && i == 0) {
      resample_start_fraction = start_fraction;
    }
    double resample_end_fraction = 1.0;
    if (end_fraction < 1.0 &&
        i + 1 == (lane_ids.empty() ? all_lane_info.size() : lane_ids.size())) {
      resample_end_fraction = end_fraction;
    }

    if (resample_end_fraction < resample_start_fraction) continue;
    if (resample_end_fraction == resample_start_fraction) {
      const mapping::LanePoint lane_point =
          mapping::LanePoint(lane_id, resample_end_fraction);
      const double d_sqr =
          ComputeLanePointPos(psmm, lane_point).DistanceSquareTo(query_point) +
          heading_penalty_weight *
              Sqr(NormalizeAngle(
                  NormalizeAngle2D(ComputeLanePointTangent(psmm, lane_point)) -
                  heading));
      if (min_d_sqr > d_sqr) {
        min_d_sqr = d_sqr;
        min_d_lane_id = lane_id;
        min_d_cumulative_length = resample_end_fraction;
        min_d_closest_point_on_lane = ComputeLanePointPos(psmm, lane_point);
      }
      continue;
    }

    bool resampled =
        resample_start_fraction > 0.0 || resample_end_fraction < 1.0;
    std::vector<double> resampled_cumulative_lengths;
    const auto& points =
        resampled ? mapping::ResampleLanePoints(
                        lane_info, resample_start_fraction,
                        resample_end_fraction, &resampled_cumulative_lengths)
                  : lane_info.points();

    const auto& cumulative_lengths =
        resampled ? resampled_cumulative_lengths
                  : lane_info.center_line().GetAccuLength();
    ;

    for (int j = 0; j + 1 < points.size(); ++j) {
      const Vec2d p0(points[j]);
      const Vec2d p1(points[j + 1]);

      const Vec2d segment = p1 - p0;
      const double l_sqr = segment.squaredNorm();
      const Vec2d query_segment = query_point - p0;
      const double projection = query_segment.dot(segment) / l_sqr;
      double d_sqr;
      double cumulative_length;
      Vec2d closest_point;
      if (projection < 0.0) {
        d_sqr = p0.DistanceSquareTo(query_point);
        closest_point = p0;
        cumulative_length = cumulative_lengths[j];
      } else if (projection > 1.0) {
        d_sqr = p1.DistanceSquareTo(query_point);
        closest_point = p1;
        cumulative_length = cumulative_lengths[j + 1];
      } else {
        d_sqr = Sqr(query_segment.CrossProd(segment.normalized()));
        closest_point = Lerp(p0, p1, projection);
        cumulative_length =
            Lerp(cumulative_lengths[j], cumulative_lengths[j + 1], projection);
      }

      d_sqr += heading_penalty_weight *
               Sqr(NormalizeAngle(segment.FastAngle() - heading));
      if (min_d_sqr > d_sqr) {
        min_d_sqr = d_sqr;
        min_d_lane_id = lane_id;
        min_d_cumulative_length = cumulative_length;
        min_d_closest_point_on_lane = closest_point;
      }
    }
  }

  if (!min_d_lane_id.has_value()) {
    return absl::NotFoundError("Could not find any valid lane.");
  }

  if (closest_point_on_lane != nullptr) {
    *closest_point_on_lane = min_d_closest_point_on_lane;
  }

  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(min_d_lane_info, psmm, *min_d_lane_id,
                                  mapping::LanePoint(*min_d_lane_id, 0.0));

  double fraction = min_d_cumulative_length / min_d_lane_info.curve_length();

  if (!lane_ids.empty() && *min_d_lane_id == lane_ids[0] &&
      fraction < start_fraction) {
    fraction = start_fraction;
  }
  return mapping::LanePoint(*min_d_lane_id, fraction);
}

std::vector<std::pair<double, mapping::LanePoint>>
FindCloseLanePointsAndDistanceToSmoothPointWithHeadingBoundAmongLanesAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    double heading, double heading_penalty_weight,
    double spatial_distance_threshold, double angle_error_threshold) {
  std::vector<std::pair<double, mapping::LanePoint>> close_points;
  const auto& all_lane_info = psmm.map_ptr()->lanes();
  for (const auto& lane_info : all_lane_info) {
    double min_sqr_error = std::numeric_limits<double>::infinity();
    double min_d_sqr{0}, min_angle_error{0}, min_d_cumulative_length{0};

    if (!lane_info->points().size() ||
        (spatial_distance_threshold < std::numeric_limits<double>::infinity() &&
         std::max((query_point - lane_info->points()[0]).squaredNorm(),
                  (query_point - lane_info->points().back()).squaredNorm()) >
             Sqr(spatial_distance_threshold + mapping::kEpsilon +
                 lane_info->curve_length()))) {
      continue;
    }

    for (int j = 0; j + 1 < lane_info->points().size(); ++j) {
      const Vec2d p0(lane_info->points()[j]);
      const Vec2d p1(lane_info->points()[j + 1]);

      const Vec2d segment = p1 - p0;
      const double l_sqr = segment.squaredNorm();
      const Vec2d query_segment = query_point - p0;
      const double projection = query_segment.dot(segment) / l_sqr;
      double d_sqr;
      double cumulative_length;
      Vec2d closest_point;
      if (projection < 0.0) {
        d_sqr = p0.DistanceSquareTo(query_point);
        closest_point = p0;
        cumulative_length = lane_info->center_line().GetAccuLength()[j];
      } else if (projection > 1.0) {
        d_sqr = p1.DistanceSquareTo(query_point);
        closest_point = p1;
        cumulative_length = lane_info->center_line().GetAccuLength()[j + 1];
      } else {
        d_sqr = Sqr(query_segment.CrossProd(segment.normalized()));
        closest_point = Lerp(p0, p1, projection);
        cumulative_length =
            Lerp(lane_info->center_line().GetAccuLength()[j],
                 lane_info->center_line().GetAccuLength()[j + 1], projection);
      }

      const double angle_error = NormalizeAngle(segment.FastAngle() - heading);
      const double error = d_sqr + heading_penalty_weight * Sqr(angle_error);

      if (min_sqr_error > error) {
        min_sqr_error = error;
        min_d_sqr = d_sqr;
        min_angle_error = std::abs(angle_error);
        min_d_cumulative_length = cumulative_length;
      }
    }

    if (min_sqr_error < std::numeric_limits<double>::infinity() &&
        min_d_sqr < Sqr(spatial_distance_threshold) &&
        min_angle_error < angle_error_threshold) {
      close_points.emplace_back(
          min_sqr_error,
          mapping::LanePoint(lane_info->id(), min_d_cumulative_length /
                                                  lane_info->curve_length()));
    }
  }

  std::sort(
      close_points.begin(), close_points.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
  return close_points;
}

std::vector<mapping::LanePoint>
FindCloseLanePointsToSmoothPointWithHeadingBoundAmongLanesAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    double heading, double heading_penalty_weight,
    double spatial_distance_threshold, double angle_error_threshold) {
  auto close_points =
      FindCloseLanePointsAndDistanceToSmoothPointWithHeadingBoundAmongLanesAtLevel(
          psmm, query_point, heading, heading_penalty_weight,
          spatial_distance_threshold, angle_error_threshold);
  std::vector<mapping::LanePoint> points;
  points.reserve(close_points.size());
  for (const auto& [err, pt] : close_points) {
    points.emplace_back(pt);
  }
  return points;
}

bool IsPointOnLanePathAtLevel(const PlannerSemanticMapManager& psmm,
                              const Vec2d& query_point,
                              const mapping::LanePath& lane_path,
                              double* arc_len_on_lane_path,
                              double lateral_error_buffer) {
  Vec2d closest_pt;
  const auto lane_point =
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          psmm, query_point, lane_path, 0.0, 0.0, &closest_pt);
  if (!lane_point.ok()) return false;

  *arc_len_on_lane_path =
      lane_path.FirstOccurrenceOfLanePointToArclength(*lane_point);

  return query_point.DistanceSquareTo(closest_pt) < Sqr(lateral_error_buffer);
}

}  // namespace e2e_noa::planning
