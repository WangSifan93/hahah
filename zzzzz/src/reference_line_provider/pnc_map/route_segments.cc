//
// Copyright 2024 ZDrive.AI. All Rights Reserved.
//

/**
 * @file:
 **/

#include <algorithm>
#include "vec2d.h"
#include "math_utils.h"
#include "util.h"
#include "apps/planning/src/reference_line_provider/pnc_map/hdmap_api.h"
#include "apps/planning/src/reference_line_provider/pnc_map/route_segments.h"

namespace zark {
namespace planning {
namespace hdmap {
namespace {

// Minimum error in lane segmentation.
constexpr double kSegmentationEpsilon = 0.2;
// Minimum error in current lane segmentation.
constexpr double kCurSegmentationEpsilon = 0.55;
}  // namespace

const std::string &RouteSegments::Id() const { return id_; }

void RouteSegments::SetId(const std::string &id) { id_ = id; }

void RouteSegments::SetCanExit(bool can_exit) { can_exit_ = can_exit; }

bool RouteSegments::CanExit() const { return can_exit_; }

bool RouteSegments::StopForDestination() const { return stop_for_destination_; }

void RouteSegments::SetStopForDestination(bool stop_for_destination) {
  stop_for_destination_ = stop_for_destination;
}

void RouteSegments::SetRampConnectId(
    const std::vector<std::string> &connect_ids) {
  ramp_connect_ids_.clear();
  if (connect_ids.empty()) {
    return;
  }
  for (auto &connect_id : connect_ids) {
    ramp_connect_ids_.emplace_back(connect_id);
  }
}

bool RouteSegments::WithinLaneSegment(const LaneSegment &lane_segment,
                                      const LaneWaypoint &waypoint) {
  return waypoint.lane &&
         lane_segment.lane->lane().id().id() ==
             waypoint.lane->lane().id().id() &&
         lane_segment.start_s - kCurSegmentationEpsilon <= waypoint.s &&
         lane_segment.end_s + kCurSegmentationEpsilon >= waypoint.s;
}

bool RouteSegments::WithinLaneSegment(
    const LaneSegment &lane_segment,
    const zark::routing::LaneWaypoint &waypoint) {
  return lane_segment.lane &&
         lane_segment.lane->lane().id().id() == waypoint.id() &&
         lane_segment.start_s - kSegmentationEpsilon <= waypoint.s() &&
         lane_segment.end_s + kSegmentationEpsilon >= waypoint.s();
}

bool RouteSegments::WithinLaneSegment(
    const zark::routing::LaneSegment &lane_segment,
    const LaneWaypoint &waypoint) {
  return waypoint.lane &&
         lane_segment.id() == waypoint.lane->lane().id().id() &&
         lane_segment.start_s() - kSegmentationEpsilon <= waypoint.s &&
         lane_segment.end_s() + kSegmentationEpsilon >= waypoint.s;
}

bool RouteSegments::WithinLaneSegment(
    const zark::routing::LaneSegment &lane_segment,
    const zark::routing::LaneWaypoint &waypoint) {
  return lane_segment.id() == waypoint.id() &&
         lane_segment.start_s() - kSegmentationEpsilon <= waypoint.s() &&
         lane_segment.end_s() + kSegmentationEpsilon >= waypoint.s();
}

bool RouteSegments::Stitch(const RouteSegments &other) {
  auto first_waypoint = FirstWaypoint();
  bool has_overlap = IsWaypointOnSegment(other.FirstWaypoint());
  if (other.IsWaypointOnSegment(first_waypoint)) {
    auto iter = other.begin();
    while (iter != other.end() && !WithinLaneSegment(*iter, first_waypoint)) {
      ++iter;
    }
    begin()->start_s = std::min(begin()->start_s, iter->start_s);
    begin()->end_s = std::max(begin()->end_s, iter->end_s);
    insert(begin(), other.begin(), iter);
    has_overlap = true;
  }
  auto last_waypoint = LastWaypoint();
  if (other.IsWaypointOnSegment(last_waypoint)) {
    auto iter = other.rbegin();
    while (iter != other.rend() && !WithinLaneSegment(*iter, last_waypoint)) {
      ++iter;
    }
    back().start_s = std::min(back().start_s, iter->start_s);
    back().end_s = std::max(back().end_s, iter->end_s);
    insert(end(), iter.base(), other.end());
    has_overlap = true;
  }
  return has_overlap;
}

const LaneWaypoint &RouteSegments::RouteEndWaypoint() const {
  return route_end_waypoint_;
}

bool RouteSegments::IsOnSegment() const { return is_on_segment_; }

void RouteSegments::SetIsOnSegment(bool on_segment) {
  is_on_segment_ = on_segment;
}

bool RouteSegments::IsNeighborSegment() const { return is_neighbor_; }

void RouteSegments::SetIsNeighborSegment(bool is_neighbor) {
  is_neighbor_ = is_neighbor;
}

void RouteSegments::SetRouteEndWaypoint(const LaneWaypoint &waypoint) {
  route_end_waypoint_ = waypoint;
}

LaneWaypoint RouteSegments::FirstWaypoint() const {
  return LaneWaypoint(front().lane, front().start_s, 0.0);
}

LaneWaypoint RouteSegments::LastWaypoint() const {
  return LaneWaypoint(back().lane, back().end_s, 0.0);
}

void RouteSegments::SetProperties(const RouteSegments &other) {
  route_end_waypoint_ = other.RouteEndWaypoint();
  can_exit_ = other.CanExit();
  is_on_segment_ = other.IsOnSegment();
  next_action_ = other.NextAction();
  previous_action_ = other.PreviousAction();
  id_ = other.Id();
  stop_for_destination_ = other.StopForDestination();
  neibor_num_ = other.getNeighborNum();
  diff_road_dis_ = other.GetDiffRoadDis();
  next_separat_direction_ = other.GetSepratDirection();
  segment_type_ = other.GetSegmentType();
  ramp_connect_ids_.clear();
  for (auto &connect_id : other.GetRampConnectId()) {
    ramp_connect_ids_.emplace_back(connect_id);
  }
}

double RouteSegments::Length(const RouteSegments &segments) {
  double s = 0.0;
  for (const auto &seg : segments) {
    s += seg.Length();
  }
  return s;
}

bool RouteSegments::GetProjection(const PointENU &point_enu, SLPoint *sl_point,
                                  LaneWaypoint *waypoint) const {
  return GetProjection(::math::Vec2d{point_enu.x(), point_enu.y()}, sl_point,
                       waypoint);
}

bool RouteSegments::IsConnectedSegment(const RouteSegments &other) const {
  if (empty() || other.empty()) {
    return false;
  }
  if (IsWaypointOnSegment(other.FirstWaypoint())) {
    return true;
  }
  if (IsWaypointOnSegment(other.LastWaypoint())) {
    return true;
  }
  if (other.IsWaypointOnSegment(FirstWaypoint())) {
    return true;
  }
  if (other.IsWaypointOnSegment(LastWaypoint())) {
    return true;
  }
  return false;
}

bool RouteSegments::Shrink(const ::math::Vec2d &point,
                           const double look_backward,
                           const double look_forward) {
  SLPoint sl_point;
  LaneWaypoint waypoint;
  if (!GetProjection(point, &sl_point, &waypoint)) {
    AERROR << "failed to project " << point.DebugString() << " to segment";
    return false;
  }
  return Shrink(sl_point.s(), look_backward, look_forward);
}

bool RouteSegments::Shrink(const double s, const double look_backward,
                           const double look_forward) {
  LaneWaypoint waypoint;
  if (!GetWaypoint(s, &waypoint)) {
    return false;
  }
  return Shrink(s, waypoint, look_backward, look_forward);
}

bool RouteSegments::Shrink(const double s, const LaneWaypoint &waypoint,
                           const double look_backward,
                           const double look_forward) {
  double acc_s = 0.0;
  auto iter = begin();
  while (iter != end() && acc_s + iter->Length() < s - look_backward) {
    acc_s += iter->Length();
    ++iter;
  }
  if (iter == end()) {
    return true;
  }
  iter->start_s =
      std::max(iter->start_s, s - look_backward - acc_s + iter->start_s);
  if (iter->Length() < kSegmentationEpsilon) {
    ++iter;
  }
  erase(begin(), iter);

  iter = begin();
  acc_s = 0.0;
  while (iter != end() && !WithinLaneSegment(*iter, waypoint)) {
    ++iter;
  }
  if (iter == end()) {
    return true;
  }
  acc_s = iter->end_s - waypoint.s;
  if (acc_s >= look_forward) {
    iter->end_s = waypoint.s + look_forward;
    ++iter;
    erase(iter, end());
    return true;
  }
  ++iter;
  while (iter != end() && acc_s + iter->Length() < look_forward) {
    acc_s += iter->Length();
    ++iter;
  }
  if (iter == end()) {
    return true;
  }
  iter->end_s = std::min(iter->end_s, look_forward - acc_s + iter->start_s);
  erase(iter + 1, end());
  return true;
}

bool RouteSegments::GetWaypoint(const double s, LaneWaypoint *waypoint) const {
  double accumulated_s = 0.0;
  bool has_projection = false;
  for (auto iter = begin(); iter != end();
       accumulated_s += (iter->end_s - iter->start_s), ++iter) {
    if (accumulated_s - kSegmentationEpsilon < s &&
        s < accumulated_s + iter->end_s - iter->start_s +
                kSegmentationEpsilon) {
      waypoint->lane = iter->lane;
      waypoint->s = s - accumulated_s + iter->start_s;
      if (waypoint->s < iter->start_s) {
        waypoint->s = iter->start_s;
      } else if (waypoint->s > iter->end_s) {
        waypoint->s = iter->end_s;
      }
      has_projection = true;
      break;
    }
  }
  return has_projection;
}

bool RouteSegments::GetProjection(const ::math::Vec2d &point, SLPoint *sl_point,
                                  LaneWaypoint *waypoint) const {
  double min_l = std::numeric_limits<double>::infinity();
  double accumulated_s = 0.0;
  bool has_projection = false;
  for (auto iter = begin(); iter != end();
       accumulated_s += (iter->end_s - iter->start_s), ++iter) {
    double lane_s = 0.0;
    double lane_l = 0.0;
    if (!iter->lane->GetProjection(math::Vec2d(point.x(), point.y()), &lane_s,
                                   &lane_l)) {
      AERROR << "Failed to get projection from point " << point.DebugString()
             << " on lane " << iter->lane->lane().id().id();
      return false;
    }
    if (lane_s < iter->start_s - kSegmentationEpsilon ||
        lane_s > iter->end_s + kSegmentationEpsilon) {
      continue;
    }
    if (std::fabs(lane_l) < min_l) {
      has_projection = true;
      lane_s = std::max(iter->start_s, lane_s);
      lane_s = std::min(iter->end_s, lane_s);
      min_l = std::fabs(lane_l);
      sl_point->set_l(lane_l);
      sl_point->set_s(lane_s - iter->start_s + accumulated_s);
      waypoint->lane = iter->lane;
      waypoint->s = lane_s;
    }
  }
  return has_projection;
}

void RouteSegments::SetPreviousAction(zark::routing::ChangeLaneType action) {
  previous_action_ = action;
}

zark::routing::ChangeLaneType RouteSegments::PreviousAction() const {
  return previous_action_;
}

void RouteSegments::SetNextAction(zark::routing::ChangeLaneType action) {
  next_action_ = action;
}

zark::routing::ChangeLaneType RouteSegments::NextAction() const {
  return next_action_;
}

bool RouteSegments::IsWaypointOnSegment(const LaneWaypoint &waypoint) const {
  for (auto iter = begin(); iter != end(); ++iter) {
    if (WithinLaneSegment(*iter, waypoint)) {
      return true;
    }
  }
  return false;
}

bool RouteSegments::CanDriveFrom(const LaneWaypoint &waypoint,
                                 const bool &is_online_map) const {
  auto point = waypoint.lane->GetSmoothPoint(waypoint.s);

  // 0 if waypoint is on segment, ok
  if (IsWaypointOnSegment(waypoint)) {
    return true;
  }

  // 1. should have valid projection.
  LaneWaypoint segment_waypoint;
  SLPoint route_sl;
  bool has_projection = GetProjection(::math::Vec2d(point.x(), point.y()),
                                      &route_sl, &segment_waypoint);
  if (!has_projection) {
    AERROR << "No projection from waypoint: " << waypoint.DebugString();
    return false;
  }
  static constexpr double kMaxLaneWidth = 10.0;
  if (std::fabs(route_sl.l()) > 2 * kMaxLaneWidth) {
    return false;
  }

  // 2. heading should be the same.
  double waypoint_heading = waypoint.lane->Heading(waypoint.s);
  double segment_heading = segment_waypoint.lane->Heading(segment_waypoint.s);
  double heading_diff = ::math::AngleDiff(waypoint_heading, segment_heading);
  if (std::fabs(heading_diff) > M_PI / 2) {
    ADEBUG << "Angle diff too large:" << heading_diff;
    return false;
  }

  // 3. the waypoint and the projected lane should not be separated apart.
  hdmap_new::LaneSampleAssociation waypoint_width =
      waypoint.lane->GetWidth(waypoint.s / kCentimeterToMetre);
  hdmap_new::LaneSampleAssociation segment_width =
      segment_waypoint.lane->GetWidth(segment_waypoint.s);
  auto segment_projected_point =
      segment_waypoint.lane->GetSmoothPoint(segment_waypoint.s);
  double dist = ::util::DistanceXY(point, segment_projected_point);
  if (!is_online_map) {
    const double kLaneSeparationDistance = 2.0;  // 0.3;
    if (route_sl.l() < 0) {                      // waypoint at right side
      if (dist > waypoint_width.left_width_cm() * kCentimeterToMetre +
                     segment_width.right_width_cm() * kCentimeterToMetre +
                     kLaneSeparationDistance) {
        AERROR << "waypoint is too far to reach: " << dist;
        return false;
      }
    } else {  // waypoint at left side
      if (dist > waypoint_width.right_width_cm() * kCentimeterToMetre +
                     segment_width.left_width_cm() * kCentimeterToMetre +
                     kLaneSeparationDistance) {
        AERROR << "waypoint is too far to reach: " << dist;
        AERROR << "waypoint_width.right_width_cm()"
               << waypoint_width.right_width_cm();
        AERROR << "segment_width.left_width_cm()"
               << segment_width.left_width_cm();
        AERROR << "kLaneSeparationDistance" << kLaneSeparationDistance;
        return false;
      }
    }
  }
  AINFO << "waypoint to reach: " << dist;
  return true;
}

bool RouteSegments::isCloseNextPassage() const {
  if (is_on_segment_ ||
      previous_action_ == zark::routing::ChangeLaneType::FORWARD) {
    return true;
  }
  return previous_action_ == next_action_;
}

}  // namespace hdmap
}  // namespace planning
}  // namespace zark
