#include "object/plan_passage_filter.h"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "math/frenet_common.h"
#include "math/geometry/polygon2d.h"
#include "math/util.h"
#include "perception.pb.h"
#include "prediction/prediction_util.h"
#include "util/status_macros.h"

namespace e2e_noa {
namespace planning {

namespace {

double GetDistanceBuffer(const PlannerObject& object) {
  switch (object.type()) {
    case OT_UNKNOWN_STATIC:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.8;
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return object.is_stationary() ? 0.8 : 1.2;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

std::pair<double, double> FindMaxRangeOfL(const PlanPassage& plan_passage,
                                          const PathSlBoundary& sl_boudnary,
                                          double min_s, double max_s) {
  static constexpr double kSampleStep = 0.5;
  double min_lat = std::numeric_limits<double>::infinity();
  double max_lat = -std::numeric_limits<double>::infinity();
  auto [r, l] = sl_boudnary.QueryOptBoundaryL(max_s);
  min_lat = Min(min_lat, r, l);
  max_lat = Max(max_lat, r, l);
  for (double s = min_s; s < max_s; s += kSampleStep) {
    const auto [r, l] = sl_boudnary.QueryOptBoundaryL(s);
    const auto lane_boundary_info =
        plan_passage.QueryEnclosingLaneBoundariesAtS(s);
    min_lat = Min(min_lat, r, lane_boundary_info.right->lat_offset);
    max_lat = Max(max_lat, l, lane_boundary_info.left->lat_offset);
  }
  return {min_lat, max_lat};
}

bool HasLateralOverlap(const PlanPassage& plan_passage,
                       const Polygon2d& contour, double boundary_min_l,
                       double boundary_max_l) {
  CHECK_GE(boundary_max_l, boundary_min_l);

  double contour_min_l = std::numeric_limits<double>::infinity();
  double contour_max_l = -std::numeric_limits<double>::infinity();
  bool has_projection = false;
  for (const auto& pt : contour.points()) {
    ASSIGN_OR_CONTINUE(
        const auto offset,
        plan_passage.QueryLaterallyUnboundedFrenetCoordinateAt(pt));
    has_projection = true;
    UpdateMin(offset.l, &contour_min_l);
    UpdateMax(offset.l, &contour_max_l);
  }
  if (!has_projection) return false;

  return contour_min_l <= boundary_max_l && contour_max_l >= boundary_min_l;
}

bool TrajectoryMaybeHasOverlap(const PlanPassage& plan_passage,
                               const PathSlBoundary& sl_boundary,
                               const prediction::PredictedTrajectory& traj,
                               double padding) {
  const auto& points = traj.points();
  for (int i = 0, n = points.size(); i < n; ++i) {
    const auto projection_or =
        plan_passage.QueryLaterallyUnboundedFrenetCoordinateAt(points[i].pos());
    if (!projection_or.ok()) {
      continue;
    }
    const auto& projection = *projection_or;

    double dist = 0.0;
    if (projection.s < plan_passage.front_s()) {
      dist = plan_passage.front_s() - projection.s;
    } else if (projection.s > plan_passage.end_s()) {
      dist = projection.s - plan_passage.end_s();
    } else {
      const auto [sl_right_l, sl_left_l] =
          sl_boundary.QueryOptBoundaryL(projection.s);
      const auto lane_boundary_info =
          plan_passage.QueryEnclosingLaneBoundariesAtS(projection.s);
      const auto right_l =
          std::fmin(sl_right_l, lane_boundary_info.right->lat_offset);
      const auto left_l =
          std::fmax(sl_left_l, lane_boundary_info.left->lat_offset);
      if (projection.l < right_l) {
        dist = right_l - projection.l;
      } else if (projection.l > left_l) {
        dist = projection.l - left_l;
      } else {
        dist = 0.0;
      }
    }

    if (dist < padding) return true;

    for (++i; i < n && dist > padding; ++i) {
      dist -= std::abs(points[i].s() - points[i - 1].s());
    }
    --i;
  }
  return false;
}

}  // namespace

FilterReason::Type PlanPassageFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  const auto& contour = object.contour();

  const double circle_radius = contour.CircleRadius();
  const auto obj_center = contour.CircleCenter();

  const double distance_buffer = GetDistanceBuffer(object);

  const double padding = circle_radius + distance_buffer;
  if (prediction::IsStationaryTrajectory(traj)) {
    const auto projection_or =
        plan_passage_->QueryLaterallyUnboundedFrenetCoordinateAt(
            contour.CircleCenter());
    if (!projection_or.ok()) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_PLAN_PASSAGE;
    }
    const auto& projection = *projection_or;
    if (projection.s < plan_passage_->front_s() - padding ||
        projection.s > plan_passage_->end_s() + padding) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_PLAN_PASSAGE;
    }

    const auto station_index =
        plan_passage_->FindNearestStationIndex(obj_center);
    const auto& station = plan_passage_->station(station_index);
    const auto boundaries = station.boundaries();
    const double right_curb = boundaries.front().lat_offset;
    const double left_curb = boundaries.back().lat_offset;

    if (projection.l < right_curb - padding ||
        projection.l > left_curb + padding) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_PLAN_PASSAGE;
    }

    const auto [min_right_l, max_left_l] = FindMaxRangeOfL(
        *plan_passage_, *sl_boundary_, projection.s - circle_radius,
        projection.s + circle_radius);
    CHECK_LE(min_right_l, max_left_l);
    if (projection.l < min_right_l - padding ||
        projection.l > max_left_l + padding) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_SL_BOUNDARY;
    }

    if (!HasLateralOverlap(*plan_passage_, contour,
                           min_right_l - distance_buffer,
                           max_left_l + distance_buffer)) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_SL_BOUNDARY;
    }
    return FilterReason::NONE;
  }

  if (!TrajectoryMaybeHasOverlap(*plan_passage_, *sl_boundary_, traj,
                                 padding)) {
    return FilterReason::TRAJECTORY_NOT_ON_SL_BOUNDARY;
  }

  return FilterReason::NONE;
}
}  // namespace planning
}  // namespace e2e_noa
