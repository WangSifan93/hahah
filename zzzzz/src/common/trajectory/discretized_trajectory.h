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
 * @discretized_trajectory.h
 **/

#pragma once

#include <vector>

#include "apps/planning/src/common/log.h"
#include "vec2d.h"
#include "pnc_point.h"

namespace zark {
namespace planning {

class DiscretizedTrajectory : public std::vector<::common::TrajectoryPoint> {
 public:
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */

  explicit DiscretizedTrajectory(
      const std::vector<::common::TrajectoryPoint>& trajectory_points);

  void SetTrajectoryPoints(
      const std::vector<::common::TrajectoryPoint>& trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  virtual ::common::TrajectoryPoint StartPoint() const;

  virtual double GetTemporalLength() const;

  virtual double GetSpatialLength() const;

  virtual ::common::TrajectoryPoint EvaluateByT(
      const double relative_time, const bool is_zoh = false) const;

  virtual ::common::TrajectoryPoint EvaluateByS(
      const double s, const bool is_zoh = false) const;

  virtual size_t QueryLowerBoundPoint(const double relative_time,
                                      const double epsilon = 1.0e-5) const;

  virtual size_t QueryNearestPoint(const ::math::Vec2d& position) const;

  size_t QueryNearestPointWithBuffer(const ::math::Vec2d& position,
                                     const double buffer) const;

  virtual void AppendTrajectoryPoint(
      const ::common::TrajectoryPoint& trajectory_point);

  void PrependTrajectoryPoints(
      const std::vector<::common::TrajectoryPoint>& trajectory_points) {
    if (!empty() && trajectory_points.size() > 1) {
      if (!(trajectory_points.back().relative_time() <
            front().relative_time())) {
        clear();
      }
    }
    insert(begin(), trajectory_points.begin(), trajectory_points.end());
  }

  const ::common::TrajectoryPoint& TrajectoryPointAt(const size_t index) const;

  size_t NumOfPoints() const;

  virtual void Clear();

  /**
   * @brief get the projection of the trajectory
   * @param position
   * @return projection path point
   */
  ::common::PathPoint Projection(const ::math::Vec2d& position) const;
};

/**
 * @brief compute the projection s of the trajectory point
 * @param x position x
 * @param y position y
 * @param p trajetory point
 * @return projection s and l
 */
std::pair<double, double> ComputePositionProjection(
    const double x, const double y, const ::common::TrajectoryPoint& p);

inline size_t DiscretizedTrajectory::NumOfPoints() const { return size(); }

inline void DiscretizedTrajectory::Clear() { clear(); }

}  // namespace planning
}  // namespace zark
