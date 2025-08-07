/******************************************************************************
 * Copyright 2017 The zpilot Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file discretized_trajectory.cc
 **/

#include "apps/planning/src/common/trajectory/discretized_trajectory.h"

#include <algorithm>
#include <limits>

#include "apps/planning/src/common/log.h"
#include "linear_interpolation.h"

namespace zark {
namespace planning {

using ::common::TrajectoryPoint;
using ::math::Vec2d;

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points)
    : std::vector<TrajectoryPoint>(trajectory_points) {
  ACHECK(!trajectory_points.empty())
      << "trajectory_points should NOT be empty()";
}

TrajectoryPoint DiscretizedTrajectory::EvaluateByT(const double relative_time,
                                                   const bool is_zoh) const {
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);

  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    AWARN << "When evaluate trajectory, relative_time(" << relative_time
          << ") is too large";
    return back();
  }
  TrajectoryPoint result = ::math::InterpolateUsingLinearApproximationByT(
      *(it_lower - 1), *it_lower, relative_time);
  if (is_zoh) {
    result.set_a((it_lower - 1)->a());
    result.set_steer((it_lower - 1)->steer());
  }
  return result;
}

TrajectoryPoint DiscretizedTrajectory::EvaluateByS(const double s,
                                                   const bool is_zoh) const {
  auto comp = [](const TrajectoryPoint& p, const double s) {
    return p.path_point().s() < s;
  };

  auto it_lower = std::lower_bound(begin(), end(), s, comp);

  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    AWARN << "When evaluate trajectory, s(" << s << ") is too large";
    return back();
  }
  TrajectoryPoint result = ::math::InterpolateUsingLinearApproximationByS(
      *(it_lower - 1), *it_lower, s);
  if (is_zoh) {
    result.set_a((it_lower - 1)->a());
    result.set_steer((it_lower - 1)->steer());
  }
  return result;
}

size_t DiscretizedTrajectory::QueryLowerBoundPoint(const double relative_time,
                                                   const double epsilon) const {
  ACHECK(!empty());

  if (relative_time >= back().relative_time()) {
    return size() - 1;
  }
  auto func = [&epsilon](const TrajectoryPoint& tp,
                         const double relative_time) {
    return tp.relative_time() + epsilon < relative_time;
  };
  auto it_lower = std::lower_bound(begin(), end(), relative_time, func);
  return std::distance(begin(), it_lower);
}

size_t DiscretizedTrajectory::QueryNearestPoint(
    const ::math::Vec2d& position) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const ::math::Vec2d curr_point(data()[i].path_point().x(),
                                   data()[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
    const ::math::Vec2d& position, const double buffer) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const ::math::Vec2d curr_point(data()[i].path_point().x(),
                                   data()[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min + buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {
  if (!empty()) {
    CHECK_GT(trajectory_point.relative_time(), back().relative_time());
  }
  push_back(trajectory_point);
}

const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const size_t index) const {
  CHECK_LT(index, NumOfPoints());
  return data()[index];
}

TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  ACHECK(!empty());
  return front();
}

double DiscretizedTrajectory::GetTemporalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().relative_time() - front().relative_time();
}

double DiscretizedTrajectory::GetSpatialLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().path_point().s() - front().path_point().s();
}

::common::PathPoint DiscretizedTrajectory::Projection(
    const ::math::Vec2d& position) const {
  const double s =
      ComputePositionProjection(position.x(), position.y(),
                                data()[QueryNearestPoint(position)])
          .first;
  auto it_lower = std::lower_bound(
      begin(), end(), s, [](const TrajectoryPoint& p, const double s) {
        return p.path_point().s() < s;
      });
  if (it_lower == begin()) {
    return front().path_point();
  } else if (it_lower == end()) {
    AWARN << "When evaluate trajectory, s(" << s << ") is too large";
    return back().path_point();
  }
  return ::math::InterpolateUsingLinearApproximation(
      (it_lower - 1)->path_point(), it_lower->path_point(), s);
}

std::pair<double, double> ComputePositionProjection(const double x,
                                                    const double y,
                                                    const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));
  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = n.CrossProd(v);
  return frenet_sd;
}

}  // namespace planning
}  // namespace zark
