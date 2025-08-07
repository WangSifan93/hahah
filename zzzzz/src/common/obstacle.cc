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

#include "apps/planning/src/common/obstacle.h"

#include <algorithm>
#include <utility>

#include "apps/planning/src/common/base/macros.h"
#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/common/local_route/local_route.h"
#include "apps/planning/src/common/log.h"
#include "apps/planning/src/common/speed/st_boundary.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "linear_interpolation.h"
#include "util.h"

namespace {

static constexpr std::uint32_t kTrajectoryMinPoints = 2;
static constexpr std::uint32_t kMinPolygonPoints = 3;

}  // namespace

namespace zark {
namespace planning {

using planning::perception::PerceptionObstacle;
using zark::planning::common::VehicleConfigHelper;

Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle)
    : id_(id) {
  UpdateFromPerceptionObstacle(perception_obstacle);
}

Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle,
                   const ::common::Trajectory& trajectory)
    : Obstacle(id, perception_obstacle) {
  UpdateFromObstacleTrajectory(trajectory);
}

void Obstacle::UpdateFromPerceptionObstacle(
    const perception::PerceptionObstacle& perception_obstacle) {
  perception_id_ = perception_obstacle.id();
  perception_obstacle_ = perception_obstacle;
  perception_bounding_box_ = ::math::Box2d(
      {perception_obstacle.position().x(), perception_obstacle.position().y()},
      perception_obstacle.theta(), perception_obstacle.length(),
      perception_obstacle.width());
  std::vector<::math::Vec2d> polygon_points;
  if (perception_obstacle.polygon_point().size() < kMinPolygonPoints) {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  } else {
    ACHECK(perception_obstacle.polygon_point().size() >= kMinPolygonPoints)
        << "object " << id_ << "has less than 3 polygon points";
    for (const auto& point : perception_obstacle.polygon_point()) {
      polygon_points.emplace_back(point.x(), point.y());
    }
  }
  is_virtual_ = (perception_obstacle.id() < 0);
  if (!is_virtual_) {
    ACHECK(::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                &perception_polygon_))
        << "object[" << id_ << "] polygon is not a valid convex hull.\n"
        << perception_obstacle.DebugString();
  }
  speed_ = std::hypot(perception_obstacle.velocity().x(),
                      perception_obstacle.velocity().y());
}

void Obstacle::UpdateFromObstacleTrajectory(
    const ::common::Trajectory& trajectory) {
  trajectory_ = trajectory;
  auto& trajectory_points = *trajectory_.mutable_trajectory_point();
  double cumulative_s = 0.0;
  if (trajectory_points.size() > 0) {
    trajectory_points[0].mutable_path_point()->set_s(0.0);
  }
  if (trajectory_points.size() == kTrajectoryMinPoints) {
    is_static_ = true;
  }
  for (std::size_t i = 1; i < trajectory_points.size(); ++i) {
    const auto& prev = trajectory_points[i - 1];
    const auto& cur = trajectory_points[i];
    if (prev.relative_time() >= cur.relative_time()) {
      AERROR << "prediction time is not increasing."
             << "current point: " << cur.ShortDebugString()
             << "previous point: " << prev.ShortDebugString();
    }
    cumulative_s += ::util::DistanceXY(prev.path_point(), cur.path_point());
    trajectory_points[i].mutable_path_point()->set_s(cumulative_s);
  }
}

::common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time, const ::common::Trajectory& traj) {
  const auto& points = traj.trajectory_point();
  if (points.size() < kTrajectoryMinPoints) {
    return ::common::TrajectoryPoint{};
  }

  auto comp = [](const ::common::TrajectoryPoint p, const double time) {
    return p.relative_time() < time;
  };

  auto it_lower =
      std::lower_bound(points.begin(), points.end(), relative_time, comp);

  if (it_lower == points.begin()) {
    return *points.begin();
  } else if (it_lower == points.end()) {
    return *points.rbegin();
  }
  return ::math::InterpolateUsingLinearApproximationByT(
      *(it_lower - 1), *it_lower, relative_time);
}

std::list<Obstacle> Obstacle::CreateObstacles(
    const zark::prediction::proto::PredictionObjects& predictions,
    const std::int64_t plan_nano_time) {
  double dt_delay = kNanoToDecimal(
      plan_nano_time - predictions.header().common_header().timestamp_nano());
  AINFO << "Time gap between prediction and plan: " << dt_delay;
  if (dt_delay < 0.0) {
    AERROR << "Prediction time is greate planning time: " << dt_delay;
    dt_delay = 0.0;
  }

  std::list<Obstacle> obstacles{};
  if (predictions.prediction_object().empty()) {
    return obstacles;
  }
  AINFO << "Eric CreateObstacles get obs size: "
        << predictions.prediction_object_size();
  std::string all_obs{};
  for (const auto& prediction_obstacle : predictions.prediction_object()) {
    if (!IsValidPerceptionObstacle(prediction_obstacle.fusion_object())) {
      AERROR << "Invalid perception obstacle: "
             << prediction_obstacle.fusion_object().DebugString();
      continue;
    }
    planning::perception::PerceptionObstacle perception_obstacle;
    std::vector<::common::Trajectory> trajs;
    TransformPredictionMsgToTrajectory(prediction_obstacle, trajs, dt_delay);
    assert(prediction_obstacle.trajectory().size() == trajs.size());
    const auto perception_id =
        std::to_string(prediction_obstacle.fusion_object().obj_id());
    int trajectory_index = 0;
    AINFO << "Eric CreateObstacles prediction traj size: "
          << prediction_obstacle.trajectory_size()
          << ", obs_id: " << perception_id;
    AINFO << "Eric CreateObstacles traj size: " << trajs.size()
          << ", obs_id: " << perception_id;
    for (std::uint32_t idx = 0u; idx < trajs.size(); idx++) {
      const auto& traj = trajs.at(idx);
      if (!IsValidTrajectory(traj)) {
        AWARN << "invalid traj of obs " << perception_id;
        continue;
      }
      all_obs += perception_id + "_" + std::to_string(trajectory_index) + ", ";
      TransformPredictionMsgToPerception(prediction_obstacle,
                                         perception_obstacle, traj);
      obstacles.emplace_back(
          perception_id + "_" + std::to_string(trajectory_index),
          perception_obstacle, traj);
      // get/set the intent type
      const DriveStatus drive_status = ConvertDriveStatus(
          prediction_obstacle.trajectory().at(idx).intent_type());
      obstacles.back().SetDriveStatus(drive_status);

      ++trajectory_index;
    }
  }
  AINFO << "all obs: " << all_obs;
  return obstacles;
}

Obstacle Obstacle::CreateStaticVirtualObstacle(const LocalRoute& local_route,
                                               const double s,
                                               const std::string& id,
                                               const double t_end,
                                               const double t_0) {
  const double virtual_stop_wall_height = 2.0;
  perception::PerceptionObstacle perception_obstacle;
  size_t negative_id = std::hash<std::string>{}(id);
  perception_obstacle.set_id(static_cast<int32_t>(negative_id | (0x1 << 31)));
  // center point
  const auto local_route_point = local_route.GetLocalRoutePoint(s);
  perception_obstacle.mutable_position()->set_x(local_route_point.x());
  perception_obstacle.mutable_position()->set_y(local_route_point.y());
  perception_obstacle.set_theta(local_route_point.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(0);
  perception_obstacle.set_width(0);
  perception_obstacle.set_height(virtual_stop_wall_height);
  perception_obstacle.set_type(perception::Type::UNKNOWN_UNMOVABLE);
  perception_obstacle.set_tracking_time(1.0);
  auto corner_points = std::vector<::math::Vec2d>(
      4, {local_route_point.x(), local_route_point.y()});
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  // add two trajectory points for StaticVirtualObstacles created by planning
  ::common::Trajectory trajectory;
  std::vector<::common::TrajectoryPoint> points(kTrajectoryMinPoints);
  for (std::uint32_t i = 0; i < kTrajectoryMinPoints; i++) {
    ::common::TrajectoryPoint point;
    point.set_a(0.0);
    point.set_da(0.0);
    point.set_steer(0.0);
    double delt_t = (i == 0) ? t_0 : t_end;
    point.set_relative_time(delt_t);
    point.set_v(0.0);

    ::common::PathPoint path_point;
    path_point.set_x(local_route_point.x());
    path_point.set_y(local_route_point.y());
    path_point.set_kappa(0.0);
    path_point.set_dkappa(0.0);
    path_point.set_ddkappa(0.0);
    path_point.set_theta(local_route_point.heading());
    path_point.set_s(0.0);

    point.set_path_point(path_point);
    points[i] = point;
  }
  trajectory.set_trajectory_point(points);
  return Obstacle{id, perception_obstacle, trajectory};
}

bool Obstacle::IsValidPerceptionObstacle(
    const zark::sensor_fusion::Object& obstacle) {
  if (obstacle.size().x() <= 0.0) {
    AERROR << "invalid obstacle length:" << obstacle.size().x();
    return false;
  }
  if (obstacle.size().y() <= 0.0) {
    AERROR << "invalid obstacle width:" << obstacle.size().y();
    return false;
  }
  if (obstacle.size().z() <= 0.0) {
    AERROR << "invalid obstacle height:" << obstacle.size().z();
    return false;
  }
  if (obstacle.has_odom_abs_vel()) {
    if (std::isnan(obstacle.odom_abs_vel().x()) ||
        std::isnan(obstacle.odom_abs_vel().y())) {
      AERROR << "invalid obstacle velocity:"
             << obstacle.odom_abs_vel().DebugString();
      return false;
    }
  }

  auto CheckCorners = [&obstacle]() {
    auto CheckCorner = [](const zark::sensor_fusion::Vector3d& pt) {
      if (std::isnan(pt.x()) || std::isnan(pt.y())) {
        AERROR << "invalid obstacle polygon point:" << pt.DebugString();
        return false;
      }
      return true;
    };
    return CheckCorner(obstacle.corner_left_front()) &&
           CheckCorner(obstacle.corner_right_front()) &&
           CheckCorner(obstacle.corner_left_rear()) &&
           CheckCorner(obstacle.corner_right_rear());
  };

  if (!CheckCorners()) {
    return false;
  }

  return true;
}

bool Obstacle::IsValidTrajectory(const ::common::Trajectory& traj) {
  // check the size
  if (traj.trajectory_point().size() < kTrajectoryMinPoints) {
    return false;
  }

  // check the point
  for (const auto& point : traj.trajectory_point()) {
    const auto is_in_valid =
        ((!point.has_path_point()) || std::isnan(point.path_point().x()) ||
         std::isnan(point.path_point().y()) ||
         std::isnan(point.path_point().z()) ||
         std::isnan(point.path_point().kappa()) ||
         std::isnan(point.path_point().s()) ||
         std::isnan(point.path_point().dkappa()) ||
         std::isnan(point.path_point().ddkappa()) || std::isnan(point.v()) ||
         std::isnan(point.a()) || std::isnan(point.relative_time()));
    if (is_in_valid) {
      return false;
    }
  }

  return true;
}

::common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  if (trajectory_.trajectory_point().size() < kTrajectoryMinPoints) {
    ::common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(perception_obstacle_.position().x());
    point.mutable_path_point()->set_y(perception_obstacle_.position().y());
    point.mutable_path_point()->set_z(perception_obstacle_.position().z());
    point.mutable_path_point()->set_theta(perception_obstacle_.theta());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.mutable_path_point()->set_dkappa(0.0);
    point.mutable_path_point()->set_ddkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  }

  return GetPointAtTime(relative_time, trajectory_);
}

::math::Box2d Obstacle::GetBoundingBox(
    const ::common::TrajectoryPoint& point) const {
  return ::math::Box2d({point.path_point().x(), point.path_point().y()},
                       point.path_point().theta(),
                       perception_obstacle_.length(),
                       perception_obstacle_.width());
}

void Obstacle::TransformPredictionMsgToPerception(
    const zark::prediction::proto::PredictionObject& msg_obstacle,
    planning::perception::PerceptionObstacle& perception_obstacle,
    const ::common::Trajectory& traj) {
  const zark::sensor_fusion::Object& msg_perception_obstacle =
      msg_obstacle.fusion_object();
  // only compensate the velocity&position, 0.0 means get the current status of
  // obstacle
  const ::common::TrajectoryPoint& pt = GetPointAtTime(0.0, traj);
  perception_obstacle.set_timestamp(
      kNanoToDecimal(msg_obstacle.timestamp()));                 // timestamp
  perception_obstacle.set_id(msg_perception_obstacle.obj_id());  // id
  ::common::Point3D position(pt.path_point().x(), pt.path_point().y(),
                             pt.path_point().z());
  perception_obstacle.set_position(position);              // position
  perception_obstacle.set_theta(pt.path_point().theta());  // odom theta;
  ::common::Point3D vel(pt.v() * std::cos(pt.path_point().theta()),
                        pt.v() * std::sin(pt.path_point().theta()),
                        msg_perception_obstacle.odom_abs_vel().z());
  perception_obstacle.set_velocity(vel);  // velocity
  ::common::Point3D acc(msg_perception_obstacle.odom_abs_acc().x(),
                        msg_perception_obstacle.odom_abs_acc().y(),
                        msg_perception_obstacle.odom_abs_acc().z());
  perception_obstacle.set_acceleration(acc);  // acceleration
  perception_obstacle.set_length(msg_perception_obstacle.size().x());
  perception_obstacle.set_width(msg_perception_obstacle.size().y());
  perception_obstacle.set_height(msg_perception_obstacle.size().z());

  auto AddPolygonPoints = [](const zark::sensor_fusion::Vector3d& polygon_pt,
                             std::vector<Point3D>& polygon_points) {
    ::common::Point3D polygon_point(polygon_pt.x(), polygon_pt.y(),
                                    polygon_pt.z());
    polygon_points.emplace_back(polygon_point);
  };

  std::vector<::common::Point3D> polygon_points;
  AddPolygonPoints(msg_perception_obstacle.corner_left_front(), polygon_points);
  AddPolygonPoints(msg_perception_obstacle.corner_right_front(),
                   polygon_points);
  AddPolygonPoints(msg_perception_obstacle.corner_left_rear(), polygon_points);
  AddPolygonPoints(msg_perception_obstacle.corner_right_rear(), polygon_points);
  perception_obstacle.set_polygon_point(polygon_points);  // polygon_point
  // type:
  // TODO: update perception::Type and perception::SubType
  switch (msg_perception_obstacle.class_type()) {
    case zark::sensor_fusion::Object_ClassType_UNKNOWN:
      perception_obstacle.set_type(perception::Type::UNKNOWN);
      perception_obstacle.set_sub_type(perception::SubType::ST_UNKNOWN);
      break;
    case zark::sensor_fusion::Object_ClassType_CAR:
      perception_obstacle.set_type(perception::Type::VEHICLE);
      perception_obstacle.set_sub_type(perception::SubType::ST_CAR);
      break;
    case zark::sensor_fusion::Object_ClassType_BIG_VEHICLE:
      perception_obstacle.set_type(perception::Type::VEHICLE);
      perception_obstacle.set_sub_type(perception::SubType::ST_HEAVY_TRUCK);
      break;
    case zark::sensor_fusion::Object_ClassType_PEDESTRIAN:
      perception_obstacle.set_type(perception::Type::PEDESTRIAN);
      perception_obstacle.set_sub_type(perception::SubType::ST_PEDESTRIAN);
      break;
    case zark::sensor_fusion::Object_ClassType_BICYCLE:
      perception_obstacle.set_type(perception::Type::BICYCLE);
      perception_obstacle.set_sub_type(perception::SubType::ST_CYCLIST);
      break;
    case zark::sensor_fusion::Object_ClassType_CONE:
      perception_obstacle.set_type(perception::Type::UNKNOWN);
      perception_obstacle.set_sub_type(perception::SubType::ST_CONE);
      break;
    case zark::sensor_fusion::Object_ClassType_ANIMAL:
      perception_obstacle.set_type(perception::Type::UNKNOWN);
      perception_obstacle.set_sub_type(perception::SubType::ST_UNKNOWN);
      break;
    default:
      perception_obstacle.set_type(perception::Type::UNKNOWN);
      perception_obstacle.set_sub_type(perception::SubType::ST_UNKNOWN);
      break;
  }
  perception_obstacle.set_confidence(msg_perception_obstacle.confidence());
}

void Obstacle::TransformPredictionMsgToTrajectory(
    const zark::prediction::proto::PredictionObject& msg_obstacle,
    std::vector<::common::Trajectory>& trajs, const double dt_delay) {
  for (const auto& msg_traj : msg_obstacle.trajectory()) {
    ::common::Trajectory traj;
    auto& points = *traj.mutable_trajectory_point();
    for (const auto& msg_point : msg_traj.trajectory_point()) {
      ::common::TrajectoryPoint point;
      point.set_a(msg_point.a());
      point.set_da(msg_point.da());
      point.set_steer(msg_point.steer());
      point.set_relative_time(msg_point.relative_time() - dt_delay);
      point.set_v(msg_point.v());

      ::common::PathPoint path_point;
      path_point.set_x(msg_point.path_point().x());
      path_point.set_y(msg_point.path_point().y());
      path_point.set_z(msg_point.path_point().z());
      path_point.set_kappa(msg_point.path_point().kappa());
      path_point.set_dkappa(msg_point.path_point().dkappa());
      path_point.set_ddkappa(msg_point.path_point().ddkappa());
      path_point.set_theta(msg_point.path_point().theta());
      path_point.set_s(msg_point.path_point().s());

      point.set_path_point(path_point);
      points.emplace_back(point);
    }
    trajs.emplace_back(traj);
  }
}

::math::Polygon2d Obstacle::GetPolygonAtPoint(
    const ::common::TrajectoryPoint& input_point) const {
  ::math::Polygon2d polygon;
  if (!::math::Polygon2d::ComputeConvexHull(
          GetBoundingBox(input_point).GetAllCorners(), &polygon)) {
    AINFO << "polygon debug, invalid box polygon";
  }
  return polygon;
}

bool Obstacle::IsValidObstacle(
    const perception::PerceptionObstacle& perception_obstacle) {
  const double object_width = perception_obstacle.width();
  const double object_length = perception_obstacle.length();

  const double kMinObjectDimension = 1.0e-6;
  return !std::isnan(object_width) && !std::isnan(object_length) &&
         object_width > kMinObjectDimension &&
         object_length > kMinObjectDimension;
}

Obstacle::DriveStatus Obstacle::ConvertDriveStatus(
    const zark::prediction::proto::IntentTrajectory_DriveStatus drive_status) {
  using zppi = zark::prediction::proto::IntentTrajectory_DriveStatus;
  switch (drive_status) {
    case zppi::IntentTrajectory_DriveStatus_STATUS_LANE_KEEP:
      return DriveStatus::STATUS_LANE_KEEP;
    case zppi::IntentTrajectory_DriveStatus_STATUS_LANE_LEFT_CHANGE:
      return DriveStatus::STATUS_LANE_LEFT_CHANGE;
    case zppi::IntentTrajectory_DriveStatus_STATUS_LANE_RIGHT_CHANGE:
      return DriveStatus::STATUS_LANE_RIGHT_CHANGE;
    default:
      return DriveStatus::DEFAULT_STATUS;
  }
}

}  // namespace planning
}  // namespace zark
