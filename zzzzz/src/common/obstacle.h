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
 * @file
 **/

#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "apps/planning/src/common/indexed_list.h"
#include "apps/planning/src/common/indexed_ptr_list.h"
#include "apps/planning/src/common/local_view.h"
#include "apps/planning/src/common/perception_obstacle.h"
#include "apps/planning/src/common/speed/st_boundary.h"
#include "apps/planning/src/common/vehicle_config.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/planning_msgs/decision.h"
#include "apps/planning/src/planning_msgs/sl_boundary.h"
#include "box2d.h"
#include "vec2d.h"

namespace zark {
namespace planning {

class LocalRoute;

/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties.
 */

class Obstacle {
 public:
  enum DriveStatus {
    DEFAULT_STATUS = 0,
    STATUS_LANE_KEEP = 1,
    STATUS_LANE_RIGHT_CHANGE = 2,
    STATUS_LANE_LEFT_CHANGE = 3
  };

  Obstacle() = default;
  // TODO: to deprecate. we will use prediction trajectory exclusively.
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle);

  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const ::common::Trajectory& trajectory);

  void UpdateFromPerceptionObstacle(
      const perception::PerceptionObstacle& perception_obstacle);

  void UpdateFromObstacleTrajectory(const ::common::Trajectory& trajectory);

  const std::string& Id() const { return id_; }
  void SetId(const std::string& id) { id_ = id; }

  double speed() const { return speed_; }

  int32_t PerceptionId() const { return perception_id_; }

  bool IsStatic() const { return is_static_; }
  bool IsVirtual() const { return is_virtual_; }

  const ::math::Box2d& PerceptionBoundingBox() const {
    return perception_bounding_box_;
  }
  const ::math::Polygon2d& PerceptionPolygon() const {
    return perception_polygon_;
  }
  const ::common::Trajectory& Trajectory() const { return trajectory_; }
  ::common::TrajectoryPoint* AddTrajectoryPoint() {
    return trajectory_.add_trajectory_point();
  }
  bool HasTrajectory() const {
    return !(trajectory_.trajectory_point().empty());
  }

  const perception::PerceptionObstacle& Perception() const {
    return perception_obstacle_;
  }

  inline RelativeRegionType RelativeRegionToLocalRoute() const {
    return relative_region_to_local_route_;
  }

  inline void SetRelativeRegion2LocalRoute(const RelativeRegionType& type) {
    relative_region_to_local_route_ = type;
    return;
  }

  inline void SetPerceptionSlBoundary(const SLBoundary& sl_boundary) {
    sl_boundary_ = sl_boundary;
  }

  inline const SLBoundary& PerceptionSLBoundary() const { return sl_boundary_; }

  inline DriveStatus GetDriveStatus() const { return drive_status_; }
  inline void SetDriveStatus(const DriveStatus& status) {
    drive_status_ = status;
  }

  /**
   * @brief This is a helper function that can create obstacles from
   * prediction data.  The original prediction may have multiple trajectories
   * for each obstacle. But this function will create one obstacle for each
   * trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  static std::list<Obstacle> CreateObstacles(
      const zark::prediction::proto::PredictionObjects& predictions,
      const std::int64_t plan_nano_time);

  static Obstacle CreateStaticVirtualObstacle(const LocalRoute& local_route,
                                              const double s,
                                              const std::string& id,
                                              const double t_end,
                                              const double t_0 = 0.0);

  static bool IsValidPerceptionObstacle(
      const zark::sensor_fusion::Object& obstacle);

  static bool IsValidTrajectory(const ::common::Trajectory& traj);

  static ::common::TrajectoryPoint GetPointAtTime(
      const double time, const ::common::Trajectory& traj);

  void CheckLaneBlocking(const LocalRoute& local_route);

  ::common::TrajectoryPoint GetPointAtTime(const double time) const;

  ::math::Box2d GetBoundingBox(const ::common::TrajectoryPoint& point) const;

  // transform proto type prediction to perception and trajectory
  static void TransformPredictionMsgToPerception(
      const zark::prediction::proto::PredictionObject& msg_obstacle,
      planning::perception::PerceptionObstacle& perception_obstacle,
      const ::common::Trajectory& traj);

  static void TransformPredictionMsgToTrajectory(
      const zark::prediction::proto::PredictionObject& msg_obstacle,
      std::vector<::common::Trajectory>& trajs, const double dt_delay);

  static DriveStatus ConvertDriveStatus(
      const zark::prediction::proto::IntentTrajectory_DriveStatus drive_status);

  ::math::Polygon2d GetPolygonAtPoint(
      const ::common::TrajectoryPoint& point) const;

  const bool IsOncoming() const { return is_oncoming_; }
  void SetIsOncoming(const bool& is_oncoming) { is_oncoming_ = is_oncoming; }

  const bool IsApproachingStop() const { return is_approaching_stop_; }
  void SetIsApproachingStop(const bool& is_approaching_stop) { is_approaching_stop_ = is_approaching_stop; }

 private:
  bool IsValidObstacle(
      const perception::PerceptionObstacle& perception_obstacle);

 private:
  std::string id_;
  int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_virtual_ = false;
  double speed_ = 0.0;
  bool is_oncoming_ = false;
  bool is_approaching_stop_ = false;

  perception::PerceptionObstacle perception_obstacle_;
  ::common::Trajectory trajectory_;
  ::math::Box2d perception_bounding_box_;
  ::math::Polygon2d perception_polygon_;

  SLBoundary sl_boundary_;

  RelativeRegionType relative_region_to_local_route_ =
      RelativeRegionType::IDLE_TYPE;
  DriveStatus drive_status_{DriveStatus::DEFAULT_STATUS};
};

typedef IndexedList<std::string, Obstacle> IndexedObstacles;
typedef IndexedPtrList<std::string, const Obstacle*> IndexedObstaclePtrs;
typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;

}  // namespace planning
}  // namespace zark
