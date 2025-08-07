#include "router/plan_passage.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include "absl/strings/str_format.h"
#include "math/geometry/segment2d.h"
#include "math/util.h"
#include "util/status_builder.h"
#include "util/status_macros.h"

namespace e2e_noa::planning {
absl::StatusOr<std::pair<double, double>> Station::QueryCurbOffsetAt(
    double signed_lat) const {
  const double right_offset = boundaries_.front().lat_offset;
  const double left_offset = boundaries_.back().lat_offset;
  if (signed_lat < right_offset || signed_lat > left_offset) {
    return absl::OutOfRangeError(
        absl::StrFormat("Lateral offset %.2f out of range [%.2f, %.2f]!",
                        signed_lat, right_offset, left_offset));
  }

  return std::make_pair(right_offset - signed_lat, left_offset - signed_lat);
}

absl::StatusOr<BoundaryQueryResponse> Station::QueryEnclosingLaneBoundariesAt(
    double signed_lat) const {
  const double right_offset = boundaries_.front().lat_offset;
  const double left_offset = boundaries_.back().lat_offset;
  if (signed_lat < right_offset || signed_lat > left_offset) {
    return absl::OutOfRangeError(
        absl::StrFormat("Lateral offset %.2f out of range [%.2f, %.2f]!",
                        signed_lat, right_offset, left_offset));
  }

  const auto it = std::upper_bound(
      boundaries_.begin(), boundaries_.end(), signed_lat,
      [](double val, const auto& it) { return val < it.lat_offset; });

  BoundaryQueryResponse res;
  res.right = *std::prev(it);
  res.right->lat_offset -= signed_lat;
  res.left =
      it == boundaries_.end()
          ? std::nullopt
          : OptionalBoundary(
                {.type = it->type, .lat_offset = it->lat_offset - signed_lat});
  return res;
}

PlanPassage::PlanPassage(StationVector<Station> stations,
                         mapping::LanePath lane_path,
                         mapping::LanePath extend_lane_path,
                         double lane_path_start_s, bool reach_destination,
                         FrenetFrameType type, std::vector<int> change_index,
                         ad_e2e::planning::LaneSeqInfoPtr lane_seq_info)
    : stations_(std::move(stations)),
      lane_path_(std::move(lane_path)),
      extend_lane_path_(std::move(extend_lane_path)),
      beyond_lane_path_(lane_path_.back() != extend_lane_path_.back()),
      reach_destination_(reach_destination),
      lane_path_start_s_(lane_path_start_s),
      type_(type),
      lane_seq_info_(lane_seq_info),
      change_index_(change_index) {
  const int n = stations_.size();
  last_real_station_index_ = StationIndex(n - 1);
  for (const auto index : stations_.index_range()) {
    if (stations_[index].lane_id() == mapping::kInvalidElementId) {
      last_real_station_index_ = StationIndex(index.value() - 1);
      break;
    }
  }
  center_seg_inv_len_.reserve(n - 1);
  segments_.reserve(n - 1);
  for (const auto index : stations_.index_from(1)) {
    const StationIndex prev_index(index.value() - 1);
    center_seg_inv_len_.emplace_back(
        1.0 / stations_[prev_index].xy().DistanceTo(stations_[index].xy()));
    segments_.emplace_back(stations_[prev_index].xy(), stations_[index].xy());
  }
  BuildFrenetFrame();
}

absl::StatusOr<double> PlanPassage::QuerySpeedLimitAt(
    const Vec2d& point) const {
  return FindNearestStation(point).speed_limit();
}

absl::StatusOr<double> PlanPassage::QuerySpeedLimitAtS(double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }

  const auto result = BinarySearchForNearStation(s);
  return stations_[result.near_station_index].speed_limit();
}

absl::StatusOr<std::pair<double, double>> PlanPassage::QueryCurbOffsetAt(
    const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, false));
  const auto status = IsProjectionResultOnPlanPassage(projection);
  if (!status.ok()) {
    return status;
  }
  const auto& station = stations_[projection.near_station_index];

  return station.QueryCurbOffsetAt(projection.signed_l);
}

absl::StatusOr<std::pair<double, double>>
PlanPassage::QueryNearestBoundaryLateralOffset(double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }
  const auto result = BinarySearchForNearStation(s);
  const auto& near_station = stations_[result.near_station_index];
  double right_offset = std::numeric_limits<double>::lowest();
  double left_offset = std::numeric_limits<double>::max();
  for (const auto& bound : near_station.boundaries()) {
    if (bound.lat_offset < 0) {
      right_offset = std::max(right_offset, bound.lat_offset);
    }
    if (bound.lat_offset > 0) {
      left_offset = std::min(left_offset, bound.lat_offset);
    }
  }
  return std::make_pair(right_offset, left_offset);
}

absl::StatusOr<std::pair<double, double>> PlanPassage::QueryCurbOffsetAtS(
    double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }

  const auto result = BinarySearchForNearStation(s);
  return stations_[result.near_station_index].QueryCurbOffsetAt(0.0);
}

absl::StatusOr<std::pair<Vec2d, Vec2d>> PlanPassage::QueryCurbPointAt(
    const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, false));
  const auto status = IsProjectionResultOnPlanPassage(projection);
  if (!status.ok()) {
    return status;
  }
  const auto& station = stations_[projection.near_station_index];
  ASSIGN_OR_RETURN(const auto offsets,
                   station.QueryCurbOffsetAt(projection.signed_l));
  const Vec2d normal = station.tangent().Perp();

  return std::make_pair(point + normal * offsets.first,
                        point + normal * offsets.second);
}

absl::StatusOr<std::pair<Vec2d, Vec2d>> PlanPassage::QueryCurbPointAtS(
    double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }
  const auto result = BinarySearchForNearStation(s);
  const auto& near_station = stations_[result.near_station_index];
  ASSIGN_OR_RETURN(const auto offsets, near_station.QueryCurbOffsetAt(0.0));
  const auto& prev_station = stations_[result.station_index_1];
  const auto& succ_station = stations_[result.station_index_2];
  const double t =
      result.ds / (succ_station.accumulated_s() - prev_station.accumulated_s());
  const auto normal =
      Vec2d::FastUnitFromAngle(LerpAngle(prev_station.tangent().FastAngle(),
                                         succ_station.tangent().FastAngle(), t))
          .Perp();
  const auto center = Lerp(prev_station.xy(), succ_station.xy(), t);
  return std::make_pair(center + normal * offsets.first,
                        center + normal * offsets.second);
}

absl::StatusOr<BoundaryQueryResponse>
PlanPassage::QueryEnclosingLaneBoundariesAt(const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, false));
  const auto status = IsProjectionResultOnPlanPassage(projection);
  if (!status.ok()) {
    return status;
  }
  const auto& station = stations_[projection.near_station_index];

  return station.QueryEnclosingLaneBoundariesAt(projection.signed_l);
}

BoundaryQueryResponse PlanPassage::QueryEnclosingLaneBoundariesAtS(
    double s) const {
  const auto station_index = FindNearestStationIndexAtS(s);

  return stations_[station_index].QueryEnclosingLaneBoundariesAt(0.0).value();
}

absl::StatusOr<Vec2d> PlanPassage::QueryLaterallyUnboundedTangentAt(
    const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, false));
  return stations_[projection.station_index_1].tangent();
}

absl::StatusOr<Vec2d> PlanPassage::QueryTangentAt(const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, false));
  const auto status = IsProjectionResultOnPlanPassage(projection);
  if (!status.ok()) {
    return status;
  }

  return stations_[projection.station_index_1].tangent();
}

absl::StatusOr<Vec2d> PlanPassage::QueryTangentAtS(double s) const {
  ASSIGN_OR_RETURN(const auto angle, QueryTangentAngleAtS(s));
  return Vec2d::FastUnitFromAngle(angle);
}

absl::StatusOr<double> PlanPassage::QueryTangentAngleAtS(double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }

  const auto result = BinarySearchForNearStation(s);
  const auto& prev_station = stations_[result.station_index_1];
  const auto& succ_station = stations_[result.station_index_2];

  const double t =
      result.ds / (succ_station.accumulated_s() - prev_station.accumulated_s());
  return NormalizeAngle(LerpAngle(prev_station.tangent().FastAngle(),
                                  succ_station.tangent().FastAngle(), t));
}

absl::StatusOr<Vec2d> PlanPassage::QueryPointXYAtS(double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }

  const auto result = BinarySearchForNearStation(s);

  return stations_[result.station_index_1].xy() +
         stations_[result.station_index_1].tangent() * result.ds;
}

absl::StatusOr<Vec2d> PlanPassage::QueryPointXYAtSL(double s, double l) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }
  const auto result = BinarySearchForNearStation(s);
  const auto ref_tangent = stations_[result.station_index_1].tangent();
  const auto ref_xy = stations_[result.station_index_1].xy() +
                      stations_[result.station_index_1].tangent() * result.ds;

  const Vec2d normal = ref_tangent.Perp();

  return ref_xy + normal * l;
}

absl::StatusOr<StationWaypoint> PlanPassage::QueryFrenetLonOffsetAt(
    const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, false));
  const auto status = IsProjectionResultOnPlanPassage(projection);
  if (!status.ok()) {
    return status;
  }

  return StationWaypoint{
      .station_index = projection.near_station_index,
      .lon_offset = projection.accum_s -
                    stations_[projection.near_station_index].accumulated_s(),
      .accum_s = projection.accum_s};
}

absl::StatusOr<double> PlanPassage::QueryFrenetLatOffsetAt(
    const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, false));
  const auto status = IsProjectionResultOnPlanPassage(projection);
  if (!status.ok()) {
    return status;
  }

  return projection.signed_l;
}

absl::StatusOr<FrenetCoordinate>
PlanPassage::QueryLaterallyUnboundedFrenetCoordinateAt(
    const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, false));
  return FrenetCoordinate{.s = projection.accum_s, .l = projection.signed_l};
}

absl::StatusOr<FrenetCoordinate> PlanPassage::QueryUnboundedFrenetCoordinateAt(
    const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, true));

  return FrenetCoordinate{.s = projection.accum_s, .l = projection.signed_l};
}

absl::StatusOr<FrenetCoordinate> PlanPassage::QueryFrenetCoordinateAt(
    const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, false));
  const auto status = IsProjectionResultOnPlanPassage(projection);
  if (!status.ok()) {
    return status;
  }
  return FrenetCoordinate{.s = projection.accum_s, .l = projection.signed_l};
}

absl::StatusOr<FrenetBox> PlanPassage::QueryFrenetBoxAt(
    const Box2d& box, bool zone_checking) const {
  return QueryFrenetBoxAtContourPoints(box.GetCornersCounterClockwise(),
                                       zone_checking);
}

absl::StatusOr<FrenetBox> PlanPassage::QueryFrenetBoxAtContour(
    const Polygon2d& contour, bool zone_checking) const {
  return QueryFrenetBoxAtContourPoints(contour.points(), zone_checking);
}

absl::StatusOr<FrenetBox> PlanPassage::QueryFrenetBoxAtContourPoints(
    absl::Span<const Vec2d> contour_points, bool zone_checking) const {
  std::vector<FrenetCoordinate> coords;
  bool beyond_front_s = false, behind_end_s = false, left_of_right_curb = false,
       right_of_left_curb = false;
  coords.reserve(contour_points.size());
  for (const auto& pt : contour_points) {
    ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(pt, true));

    if (projection.accum_s > front_s()) beyond_front_s = true;
    if (projection.accum_s < end_s()) behind_end_s = true;

    const auto [right_curb, left_curb] =
        stations_[projection.near_station_index].QueryCurbOffsetAt(0.0).value();
    if (projection.signed_l > right_curb) left_of_right_curb = true;
    if (projection.signed_l < left_curb) right_of_left_curb = true;

    if (zone_checking) {
      coords.emplace_back(FrenetCoordinate{
          .s = std::clamp(projection.accum_s, front_s(), end_s()),
          .l = std::clamp(projection.signed_l, right_curb, left_curb)});
    } else {
      coords.emplace_back(
          FrenetCoordinate{.s = projection.accum_s, .l = projection.signed_l});
    }
  }

  if (zone_checking && (!(beyond_front_s && behind_end_s &&
                          left_of_right_curb && right_of_left_curb))) {
    return absl::NotFoundError("Box has no overlap with plan passage.");
  }
  FrenetBox frenet_box{
      .s_max = -std::numeric_limits<double>::infinity(),
      .s_min = std::numeric_limits<double>::infinity(),
      .l_max = -std::numeric_limits<double>::infinity(),
      .l_min = std::numeric_limits<double>::infinity(),
  };
  for (const auto& coord : coords) {
    frenet_box.s_max = std::max(frenet_box.s_max, coord.s);
    frenet_box.s_min = std::min(frenet_box.s_min, coord.s);
    frenet_box.l_max = std::max(frenet_box.l_max, coord.l);
    frenet_box.l_min = std::min(frenet_box.l_min, coord.l);
  }

  return frenet_box;
}

absl::StatusOr<std::vector<FrenetCoordinate>>
PlanPassage::BatchQueryFrenetCoordinates(absl::Span<const Vec2d> points) const {
  std::vector<FrenetCoordinate> frenet_points;
  frenet_points.reserve(points.size());
  for (const auto& pt : points) {
    ASSIGN_OR_RETURN(auto frenet_pt, QueryFrenetCoordinateAt(pt));
    frenet_points.push_back(frenet_pt);
  }
  return frenet_points;
}

absl::StatusOr<std::vector<std::optional<FrenetBox>>>
PlanPassage::BatchQueryFrenetBoxes(absl::Span<const Box2d> boxes,
                                   bool laterally_bounded) const {
  if (boxes.empty()) return absl::FailedPreconditionError("");

  const int n = boxes.size();
  std::vector<std::optional<FrenetBox>> results;
  results.reserve(n);

  const auto& first_box = boxes.front();
  ASSIGN_OR_RETURN(auto prev_projection,
                   ProjectPointToStations(first_box.center(), true));

  ASSIGN_OR_RETURN(auto frenet_box, QueryFrenetBoxWithinRadius(
                                        first_box, prev_projection,
                                        first_box.radius(), laterally_bounded));
  results.push_back(frenet_box);

  for (int i = 1; i < n; ++i) {
    const auto& prev_box = boxes[i - 1];
    const auto& box = boxes[i];
    constexpr double kSearchExpansionRatio = 2.0;
    const double state_search_radius =
        (prev_box.center() - box.center()).Length() * kSearchExpansionRatio;
    ASSIGN_OR_RETURN(const auto projection,
                     ProjectPointToStationsWithinRadius(
                         box.center(), prev_projection, prev_box.center(),
                         state_search_radius));
    ASSIGN_OR_RETURN(frenet_box,
                     QueryFrenetBoxWithinRadius(box, projection, box.radius(),
                                                laterally_bounded));
    results.push_back(frenet_box);
    prev_projection = projection;
  }

  return results;
}

absl::StatusOr<std::optional<FrenetBox>>
PlanPassage::QueryFrenetBoxWithinRadius(
    const Box2d& box, const ProjectionResult& center_projection,
    double search_radius, bool laterally_bounded) const {
  bool beyond_front_s = false, behind_end_s = false, left_of_right_curb = false,
       right_of_left_curb = false;
  FrenetBox frenet_box{
      .s_max = -std::numeric_limits<double>::infinity(),
      .s_min = std::numeric_limits<double>::infinity(),
      .l_max = -std::numeric_limits<double>::infinity(),
      .l_min = std::numeric_limits<double>::infinity(),
  };
  for (const auto& pt : box.GetCornersCounterClockwise()) {
    ASSIGN_OR_RETURN(auto projection,
                     ProjectPointToStationsWithinRadius(
                         pt, center_projection, box.center(), search_radius),
                     _ << "No frenet conversion is found.");

    if (projection.accum_s > front_s()) beyond_front_s = true;
    if (projection.accum_s < end_s()) behind_end_s = true;
    projection.accum_s = std::clamp(projection.accum_s, front_s(), end_s());

    const auto [right_curb, left_curb] =
        stations_[projection.near_station_index].QueryCurbOffsetAt(0.0).value();
    if (projection.signed_l > right_curb) left_of_right_curb = true;
    if (projection.signed_l < left_curb) right_of_left_curb = true;
    if (laterally_bounded) {
      projection.signed_l =
          std::clamp(projection.signed_l, right_curb, left_curb);
    }

    frenet_box.s_max = std::max(frenet_box.s_max, projection.accum_s);
    frenet_box.s_min = std::min(frenet_box.s_min, projection.accum_s);
    frenet_box.l_max = std::max(frenet_box.l_max, projection.signed_l);
    frenet_box.l_min = std::min(frenet_box.l_min, projection.signed_l);
  }
  if (!(beyond_front_s && behind_end_s && left_of_right_curb &&
        right_of_left_curb)) {
    return std::nullopt;
  }

  return frenet_box;
}

absl::StatusOr<Vec2d> PlanPassage::FindNearestPointOnCenterLine(
    const Vec2d& point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point, false));
  const auto status = IsProjectionResultOnPlanPassage(projection);
  if (!status.ok()) {
    return status;
  }
  return Lerp(stations_[projection.station_index_1].xy(),
              stations_[projection.station_index_2].xy(),
              projection.lerp_factor);
}

StationIndex PlanPassage::FindNearestStationIndex(const Vec2d& point) const {
  auto min_dis = std::numeric_limits<double>::max();
  StationIndex min_dis_index;
  for (const auto i : stations_.index_range()) {
    const auto& station = stations_[i];
    const auto dis = point.DistanceSquareTo(station.xy());
    if (dis < min_dis) {
      min_dis = dis;
      min_dis_index = i;
    }
  }
  return min_dis_index;
}

StationIndex PlanPassage::FindNearestStationIndexAtS(double s) const {
  const auto it = std::upper_bound(stations_.begin(), stations_.end(), s,
                                   [](double val, const Station& station) {
                                     return val < station.accumulated_s();
                                   });
  if (it == stations_.begin()) return StationIndex(0);
  if (it == stations_.end()) return StationIndex(stations_.size() - 1);

  return std::fabs(it->accumulated_s() - s) <
                 std::fabs(std::prev(it)->accumulated_s() - s)
             ? StationIndex(it - stations_.begin())
             : StationIndex(it - stations_.begin() - 1);
}

absl::StatusOr<PlanPassage::ProjectionResult>
PlanPassage::ProjectPointToStations(const Vec2d& point,
                                    bool allow_extrapolation) const {
  if (frenet_frame_.get() == nullptr) {
    return absl::InternalError(
        "Failed to project point to stations because of null frenet_frame_ "
        "pointer.");
  }

  FrenetCoordinate frenet_pt;
  Vec2d normal;
  std::pair<int, int> raw_index_pair;
  double lerp_factor;

  frenet_frame_->XYToSL(point, &frenet_pt, &normal, &raw_index_pair,
                        &lerp_factor);

  int nearest_index = raw_index_pair.first;
  const auto& nearest_seg = segments_[nearest_index];
  const auto prev_station_idx = StationIndex(nearest_index);
  const auto next_station_idx = StationIndex(raw_index_pair.second);
  const auto& prev_station = stations_[prev_station_idx];
  const auto& station = stations_[next_station_idx];
  double l = frenet_pt.l;
  double s =
      Lerp(prev_station.accumulated_s(), station.accumulated_s(), lerp_factor);

  if (allow_extrapolation) {
    return ProjectionResult{.station_index_1 = prev_station_idx,
                            .station_index_2 = next_station_idx,
                            .near_station_index = lerp_factor < 0.5
                                                      ? prev_station_idx
                                                      : next_station_idx,
                            .accum_s = s,
                            .signed_l = l,
                            .lerp_factor = lerp_factor};
  }

  if (lerp_factor < 0.0 && nearest_index > 1) {
    l = std::copysign(nearest_seg.start().DistanceTo(point), l);
    s = prev_station.accumulated_s();
    lerp_factor = 0.0;
  } else if (lerp_factor > 1.0 && nearest_index + 1 < stations_.size()) {
    l = std::copysign(nearest_seg.end().DistanceTo(point), l);
    s = station.accumulated_s();
    lerp_factor = 1.0;
  }

  constexpr double kEpsilon = 0.1;
  if (s < front_s() - kEpsilon || s > end_s() + kEpsilon) {
    return absl::NotFoundError("No valid projection is found.");
  }
  s = std::clamp(s, front_s(), end_s());

  return ProjectionResult{.station_index_1 = prev_station_idx,
                          .station_index_2 = next_station_idx,
                          .near_station_index = lerp_factor < 0.5
                                                    ? prev_station_idx
                                                    : next_station_idx,
                          .accum_s = s,
                          .signed_l = l,
                          .lerp_factor = lerp_factor};
}

absl::StatusOr<PlanPassage::ProjectionResult>
PlanPassage::ProjectPointToStationsWithinRadius(
    const Vec2d& point, const ProjectionResult& projection,
    const Vec2d& prev_point, double search_radius) const {
  const int start_segment_idx = projection.station_index_1.value();
  int nearest_segment_idx = start_segment_idx;
  double min_squared_dist =
      segments_[start_segment_idx].DistanceSquareTo(point);
  const double angle_diff =
      NormalizeAngle((point - prev_point).FastAngle() -
                     segments_[start_segment_idx].heading());

  std::vector<int> search_step_vec;
  constexpr double kForwardSearchThres = M_PI * 2.0 / 3.0;
  constexpr double kBackwardSearchThres = M_PI / 3.0;
  if (std::fabs(angle_diff) < kForwardSearchThres) {
    search_step_vec.push_back(1);
  }
  if (std::fabs(angle_diff) > kBackwardSearchThres) {
    search_step_vec.push_back(-1);
  }

  for (const auto step : search_step_vec) {
    for (auto idx = start_segment_idx + step;
         idx >= 0 && idx < segments_.size(); idx += step) {
      double squared_dist = segments_[idx].DistanceSquareTo(point);
      if (squared_dist < min_squared_dist) {
        min_squared_dist = squared_dist;
        nearest_segment_idx = idx;
      }
      if (std::fabs(projection.accum_s -
                    stations_[StationIndex(idx)].accumulated_s()) >
          search_radius) {
        break;
      }
    }
  }

  const auto& nearest_seg = segments_[nearest_segment_idx];
  const double prod = nearest_seg.ProductOntoUnit(point);
  const double proj = nearest_seg.ProjectOntoUnit(point) *
                      center_seg_inv_len_[nearest_segment_idx];
  const auto prev_station_idx = StationIndex(nearest_segment_idx);
  const auto next_station_idx = StationIndex(nearest_segment_idx + 1);
  const auto& prev_station = stations_[prev_station_idx];
  const auto& station = stations_[next_station_idx];

  return ProjectionResult{
      .station_index_1 = prev_station_idx,
      .station_index_2 = next_station_idx,
      .near_station_index = proj < 0.5 ? prev_station_idx : next_station_idx,
      .accum_s =
          Lerp(prev_station.accumulated_s(), station.accumulated_s(), proj),
      .signed_l = prod,
      .lerp_factor = proj};
}

absl::Status PlanPassage::IsProjectionResultOnPlanPassage(
    const ProjectionResult& res) const {
  const auto& near_station = stations_[res.near_station_index];
  if (res.signed_l < 0.0) {
    if (res.signed_l < near_station.boundaries().front().lat_offset) {
      return absl::OutOfRangeError(
          absl::StrFormat("%f is out of right lateral range %f.", res.signed_l,
                          near_station.boundaries().front().lat_offset));
    }
  } else {
    if (res.signed_l > near_station.boundaries().back().lat_offset) {
      return absl::OutOfRangeError(
          absl::StrFormat("%f is out of left lateral range %f.", res.signed_l,
                          near_station.boundaries().back().lat_offset));
    }
  }

  return absl::OkStatus();
}

bool PlanPassage::ContainIntersection(const int check_step) const {
  for (int i = 0; i < stations_.size(); i = i + check_step) {
    if (stations_[StationIndex(i)].is_in_intersection()) {
      return true;
    }
  }
  return false;
}

double PlanPassage::GetDistanceToIntersection(const Vec2d& point) const {
  double min_dis = std::numeric_limits<double>::max();
  int cur_index = FindNearestStationIndex(point).value();
  if (stations_[StationIndex(cur_index)].is_in_intersection()) {
    return 0.0;
  }
  for (int i = cur_index; i < stations_.size(); ++i) {
    const auto& station = stations_[StationIndex(i)];
    if (station.is_in_intersection()) {
      min_dis = std::min(min_dis, station.accumulated_s());
      break;
    }
  }
  return min_dis - stations_[StationIndex(cur_index)].accumulated_s();
}

double PlanPassage::GetDistanceToNonIntersection(const Vec2d& point) const {
  int cur_index = FindNearestStationIndex(point).value();
  if (!stations_[StationIndex(cur_index)].is_in_intersection()) {
    return 0.0;
  }
  for (int i = cur_index; i < stations_.size(); ++i) {
    const auto& station = stations_[static_cast<StationIndex>(i)];
    if (!station.is_in_intersection()) {
      return station.accumulated_s() -
             stations_[StationIndex(cur_index)].accumulated_s();
    }
  }
  return end_s() - stations_[StationIndex(cur_index)].accumulated_s();
}

const Station& PlanPassage::GetFirstNonIntersectionStation(
    const Vec2d& point) const {
  int cur_index = FindNearestStationIndex(point).value();
  if (!stations_[StationIndex(cur_index)].is_in_intersection()) {
    return stations_[StationIndex(cur_index)];
  }
  for (int i = cur_index; i < stations_.size(); ++i) {
    if (!stations_[static_cast<StationIndex>(i)].is_in_intersection()) {
      return stations_[static_cast<StationIndex>(i)];
    }
  }
  return stations_[StationIndex(cur_index)];
}

PlanPassage::BinarySeachResult PlanPassage::BinarySearchForNearStation(
    double s) const {
  const auto it = std::upper_bound(stations_.begin(), stations_.end(), s,
                                   [](double val, const Station& station) {
                                     return val < station.accumulated_s();
                                   });
  StationIndex prev_index;
  StationIndex next_index;
  if (it == stations_.begin()) {
    prev_index = StationIndex(0);
    next_index = StationIndex(1);
  } else if (it == stations_.end()) {
    prev_index = StationIndex(stations_.size() - 2);
    next_index = StationIndex(stations_.size() - 1);
  } else {
    prev_index = StationIndex(it - stations_.begin() - 1);
    next_index = StationIndex(it - stations_.begin());
  }

  const double ds1 = s - stations_[prev_index].accumulated_s();
  const double ds2 = stations_[next_index].accumulated_s() - s;

  return BinarySeachResult{.station_index_1 = prev_index,
                           .station_index_2 = next_index,
                           .near_station_index = std::abs(ds1) < std::abs(ds2)
                                                     ? prev_index
                                                     : next_index,
                           .ds = ds1};
}

void PlanPassage::BuildFrenetFrame() {
  std::vector<Vec2d> points;
  points.reserve(stations_.size());
  for (const auto& station : stations_) {
    points.push_back(station.xy());
  }
  switch (type_) {
    case FrenetFrameType::kBruteFroce:
      frenet_frame_ = std::make_unique<BruteForceFrenetCoordinate>(
          BuildBruteForceFrenetFrame(points, false).value());
      break;

    case FrenetFrameType::kKdTree:
      frenet_frame_ = std::make_unique<KdTreeFrenetCoordinate>(
          BuildKdTreeFrenetFrame(points, false).value());

      break;
    case FrenetFrameType::kQtfmKdTree:
      frenet_frame_ = std::make_unique<QtfmEnhancedKdTreeFrenetCoordinate>(
          BuildQtfmEnhancedKdTreeFrenetCoordinate(points, false).value());
      break;
  }

  CHECK_EQ(frenet_frame_->points().size(), stations_.size());
}

}  // namespace e2e_noa::planning
