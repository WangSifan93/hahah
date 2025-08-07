/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file corridor.cc
 **/

#include <algorithm>

#include "apps/planning/src/common/corridor.h"
#include "apps/planning/src/common/log.h"
#include "angle.h"
#include "line_segment2d.h"
#include "math_utils.h"
#include "linear_interpolation.h"

namespace zark {
namespace planning {

using ::math::LineSegment2d;
constexpr double kSampleDistance = 0.25;

Corridor::Corridor(const std::vector<CorridorPoint>& points)
    : std::vector<CorridorPoint>(std::move(points)) {
  Init();
}

void Corridor::Init() {
  num_points_ = size();
  if (num_points_ < 1) {
    AERROR << "Failed to init corridor: Corridor has less than 1 points.";
    return;
  }
  length_ = back().s - front().s;
  num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;

  last_point_index_.clear();
  last_point_index_.reserve(num_sample_points_);
  double s = front().s;
  int last_index = 0;
  for (int i = 0; i < num_sample_points_; ++i) {
    while (last_index + 1 < num_points_ && this->at(last_index + 1).s <= s) {
      ++last_index;
    }
    last_point_index_.push_back(last_index);
    s += kSampleDistance;
  }
}

bool Corridor::XYToSL(const ::math::Vec2d& xy_point,
                      ::common::SLPoint& sl_point) const {
  constexpr int kMinNumPoints = 2;
  if (num_points_ < kMinNumPoints) {
    return false;
  }

  double min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;
  for (int i = 0; i < num_points_ - 1; ++i) {
    const LineSegment2d line_seg(this->at(i).xy_ref, this->at(i + 1).xy_ref);
    const double distance = line_seg.DistanceSquareTo(xy_point);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  min_distance = std::sqrt(min_distance);
  const LineSegment2d nearest_seg(this->at(min_index).xy_ref,
                                  this->at(min_index + 1).xy_ref);
  const auto prod = nearest_seg.ProductOntoUnit(xy_point);
  const auto proj = nearest_seg.ProjectOntoUnit(xy_point);

  double s = 0.0;
  double l = 0.0;
  if (min_index == 0) {
    s = this->at(min_index).s + std::min(proj, nearest_seg.length());
    if (proj < 0) {
      l = prod;
    } else {
      l = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else if (min_index == num_points_ - 2) {
    s = this->at(min_index).s + std::max(0.0, proj);
    if (proj > 0) {
      l = prod;
    } else {
      l = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else {
    s = this->at(min_index).s +
        std::max(0.0, std::min(proj, nearest_seg.length()));
    l = (prod > 0.0 ? 1 : -1) * min_distance;
  }
  sl_point.set_s(s);
  sl_point.set_l(l);
  return true;
}

bool Corridor::SLToXY(const ::common::SLPoint& sl_point,
                      ::math::Vec2d* const xy_point) const {
  constexpr int kMinNumPoints = 2;
  if (num_points_ < kMinNumPoints) {
    return false;
  }
  CorridorPoint match_point = GetReferencePoint(sl_point.s());
  const auto angle = ::math::Angle16::from_rad(match_point.theta);
  xy_point->set_x(match_point.xy_ref.x() - ::math::sin(angle) * sl_point.l());
  xy_point->set_y(match_point.xy_ref.y() + ::math::cos(angle) * sl_point.l());
  return true;
}

CorridorPoint Corridor::GetReferencePoint(const double s) const {
  constexpr double kDeltaDisMin = 1.0e-2;
  constexpr double kMathEpsilon = 1.0e-6;
  if (s < front().s - kDeltaDisMin) {
    ADEBUG << "The requested s: " << s << " < 0.";
    return front();
  }
  if (s > back().s + kDeltaDisMin) {
    ADEBUG << "The requested s: " << s
           << " > reference line length: " << back().s;
    return back();
  }
  auto interpolate_index = GetIndexFromS(s);
  int index = interpolate_index.first;
  double offset = interpolate_index.second;
  int next_index = index + 1;
  if (next_index >= num_points_) {
    next_index = num_points_ - 1;
  }

  const auto& p0 = this->at(index);
  const auto& p1 = this->at(next_index);

  CorridorPoint match_point;
  if (offset > kMathEpsilon) {
    const double length = p1.xy_ref.DistanceTo(p0.xy_ref);
    const ::math::Vec2d unit_direction(
        (p1.xy_ref.x() - p0.xy_ref.x()) / std::max(length, kMathEpsilon),
        (p1.xy_ref.y() - p0.xy_ref.y()) / std::max(length, kMathEpsilon));
    const ::math::Vec2d delta = unit_direction * offset;
    match_point.xy_ref.set_x(p0.xy_ref.x() + delta.x());
    match_point.xy_ref.set_y(p0.xy_ref.y() + delta.y());
    match_point.theta = ::math::lerp(p0.theta, p0.s, p1.theta, p1.s, s);
    match_point.kappa = ::math::lerp(p0.kappa, p0.s, p1.kappa, p1.s, s);
  } else {
    match_point = p0;
  }
  return match_point;
}

std::pair<int, double> Corridor::GetIndexFromS(double s) const {
  if (s < front().s) {
    return std::make_pair(0, 0.0);
  }
  if (s >= length_) {
    return std::make_pair(num_points_ - 1, 0.0);
  }
  const int sample_id = static_cast<int>((s - front().s) / kSampleDistance);
  if (sample_id >= num_sample_points_) {
    return std::make_pair(num_points_ - 1, 0.0);
  }
  const int next_sample_id = sample_id + 1;
  int low = last_point_index_[sample_id];
  int high = (next_sample_id < num_sample_points_
                  ? std::min(num_points_, last_point_index_[next_sample_id] + 1)
                  : num_points_);
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (this->at(mid).s <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return std::make_pair(low, s - this->at(low).s);
}

CorridorPoint Corridor::EvaluateByS(const double s) const {
  CHECK_GT(size(), 1U);
  auto it_lower = std::lower_bound(
      begin(), end(), s,
      [](const CorridorPoint& point, const double s) { return point.s < s; });
  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    return back();
  }
  const auto& p0 = *(it_lower - 1);
  const double s0 = p0.s;
  const auto& p1 = *it_lower;
  const double s1 = p1.s;

  CorridorPoint p;
  p.s = s;
  p.l = ::math::lerp(p0.l, s0, p1.l, s1, s);
  p.s_ref = ::math::lerp(p0.s_ref, s0, p1.s_ref, s1, s);
  p.l_ref = ::math::lerp(p0.l_ref, s0, p1.l_ref, s1, s);
  p.l_left = ::math::lerp(p0.l_left, s0, p1.l_left, s1, s);
  p.l_right = ::math::lerp(p0.l_right, s0, p1.l_right, s1, s);
  p.xy_ref.set_x(::math::lerp(p0.xy_ref.x(), s0, p1.xy_ref.x(), s1, s));
  p.xy_ref.set_y(::math::lerp(p0.xy_ref.y(), s0, p1.xy_ref.y(), s1, s));
  p.theta = ::math::slerp(p0.theta, s0, p1.theta, s1, s);
  p.kappa = ::math::lerp(p0.kappa, s0, p1.kappa, s1, s);
  p.v = ::math::lerp(p0.v, s0, p1.v, s1, s);
  p.a = ::math::lerp(p0.a, s0, p1.a, s1, s);
  p.type_left = p1.type_left;
  p.type_right = p1.type_right;

  if (p.type_left == CorridorPoint::Type::CURB) {
    p.l_left = std::min(p0.l_left, p1.l_left);
  }
  if (p.type_right == CorridorPoint::Type::CURB) {
    p.l_right = std::max(p0.l_right, p1.l_right);
  }

  return p;
}

::common::FrenetPoint Corridor::ToFrenetFrame(
    const ::common::TrajectoryPoint& traj_point) const {
  constexpr int kMinNumPoints = 2;
  ACHECK(num_points_ > kMinNumPoints);

  SLPoint sl_point;
  XYToSL(math::Vec2d(traj_point.path_point().x(), traj_point.path_point().y()),
         sl_point);

  ::common::FrenetPoint frenet_pt;
  frenet_pt.t = 0.0;
  CorridorPoint corridor_point = GetReferencePoint(sl_point.s());
  math::CartesianFrenetConverter::cartesian_to_frenet(
      sl_point.s(), corridor_point.xy_ref.x(), corridor_point.xy_ref.y(),
      corridor_point.theta, corridor_point.kappa, 0.0,
      traj_point.path_point().x(), traj_point.path_point().y(), traj_point.v(),
      traj_point.a(), traj_point.path_point().theta(),
      traj_point.path_point().kappa(), &frenet_pt.s, &frenet_pt.l);

  return frenet_pt;
}

}  // namespace planning
}  // namespace zark
