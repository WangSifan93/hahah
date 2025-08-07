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
 * @file frame.cc
 **/
#include "apps/planning/src/common/frame.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "apps/planning/src/common/clock.h"
#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/common/conversion.h"
#include "apps/planning/src/common/log.h"
#include "apps/planning/src/common/util/util.h"
#include "apps/planning/src/common/vehicle_state/vehicle_state_provider.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/reference_line_provider/pnc_map/hdmap_api.h"
#include "apps/planning/src/reference_line_provider/pnc_map/path.h"
#include "apps/planning/src/reference_line_provider/pnc_map/pnc_map.h"
#include "math_utils.h"
#include "point_factory.h"
#include "status.h"
#include "string_util.h"
#include "vec2d.h"

namespace zark {
namespace planning {

using ::common::ErrorCode;
using ::common::Status;
using ::math::Box2d;
using ::math::Polygon2d;
using zark::common::Clock;

constexpr int max_frame_history_num = 1;
constexpr double oncoming_angle_threshold = std::cos(M_PI / 6.0);
static constexpr std::uint32_t kTrajectoryMinPoints = 2;

FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(max_frame_history_num) {}

Frame::Frame(uint32_t sequence_num) : sequence_num_(sequence_num) {}

Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const ::common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state)
    : sequence_num_(sequence_num),
      local_view_(local_view),
      planning_start_point_(planning_start_point),
      vehicle_state_(vehicle_state) {
  local_routes_.clear();
}

const ::common::TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

const common::VehicleState &Frame::vehicle_state() const {
  return vehicle_state_;
}

const std::list<LocalRoute> &Frame::LocalRoutes() const {
  return local_routes_;
}

LocalRoute *Frame::FindCurrentLocalRoute() {
  for (auto &local_route : local_routes_) {
    if (local_route.Lanes().GetSegmentType() ==
        hdmap::RouteSegments::SegmentType::CurrentSegment) {
      return &local_route;
    }
  }
  return nullptr;
}

Status Frame::Init(
    const common::VehicleStateProvider *vehicle_state_provider,
    const std::vector<routing::LaneWaypoint> &future_route_waypoints) {
  // TODO(QiL): refactor this to avoid redundant nullptr checks in scenarios.
  for (auto &local_route : local_routes_) {
    local_route.Init(vehicle_state_, planning_start_point_);
  }

  UpdateLeftLocalRoute();
  UpdateRightLocalRoute();

  auto status = InitFrameData(vehicle_state_provider);

  for (auto &local_route : local_routes_) {
    local_route.AddObstaclesSLBoundary(obstacles_.Items());
  }

  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();
}

Status Frame::InitFrameData(
    const common::VehicleStateProvider *vehicle_state_provider) {
  // update obstacle time horizon
  if (local_view_.prediction_obstacles->prediction_object_size() > 0) {
    auto &prediction_obstcle =
        local_view_.prediction_obstacles->prediction_object().at(0);
    obstacle_time_horizon_ = prediction_obstcle.predicted_period();
  }

  vehicle_state_ = vehicle_state_provider->vehicle_state();
  if (!util::IsVehicleStateValid(vehicle_state_)) {
    AERROR << "Adc init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "Adc init point is not set");
  }

  // TODO Remove
  const bool align_prediction_time = false;
  if (align_prediction_time) {
    auto prediction = *(local_view_.prediction_obstacles);
    AlignPredictionTime(vehicle_state_.timestamp(), &prediction);
    local_view_.prediction_obstacles->CopyFrom(prediction);
  }

  // check current local route
  LocalRoute *current_local_route = FindCurrentLocalRoute();
  if (current_local_route == nullptr) {
    const std::string msg = "current local route is not exist!";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  std::int64_t plan_nano_time = 0;
  if (PlanningGflags::enable_prediction_time_compensation) {
    plan_nano_time = vehicle_state_.nano_timestamp();
  }
  AINFO << "Eric InitFrameData get obs size: "
        << local_view_.prediction_obstacles->prediction_object_size();

  std::list<Obstacle> obstacles = Obstacle::CreateObstacles(
      *local_view_.prediction_obstacles, plan_nano_time);
  for (auto &ptr : obstacles) {
    AddObstacleRegionType(ptr, *current_local_route);
    SetIsOncoming(ptr, vehicle_state_);
    SetIsApproachingStop(ptr);
    AddObstacle(ptr);
  }

  return Status::OK();
}

uint32_t Frame::SequenceNum() const { return sequence_num_; }

std::string Frame::DebugString() const {
  return ("Frame: " + ::util::TransToString(sequence_num_));
}

void Frame::AlignPredictionTime(
    const double planning_start_time,
    zark::prediction::proto::PredictionObjects *prediction_obstacles) {
  if (!prediction_obstacles || !prediction_obstacles->has_header() ||
      !(prediction_obstacles->header().common_header().timestamp_nano() > 0)) {
    return;
  }
  double prediction_header_time =
      prediction_obstacles->header().common_header().timestamp_nano();
  for (auto &obstacle : *prediction_obstacles->mutable_prediction_object()) {
    for (auto &trajectory : *obstacle.mutable_trajectory()) {
      for (auto &point : *trajectory.mutable_trajectory_point()) {
        point.set_relative_time(prediction_header_time + point.relative_time() -
                                planning_start_time);
      }
      if (!trajectory.trajectory_point().empty() &&
          trajectory.trajectory_point().begin()->relative_time() < 0) {
        auto it = trajectory.trajectory_point().begin();
        while (it != trajectory.trajectory_point().end() &&
               it->relative_time() < 0) {
          ++it;
        }
        trajectory.mutable_trajectory_point()->erase(
            trajectory.trajectory_point().begin(), it);
      }
    }
  }
}

Obstacle *Frame::Find(const std::string &id) { return obstacles_.Find(id); }

void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

void Frame::SetIsOncoming(Obstacle &obstacle,
                          const VehicleState &vehicle_state) const {
  if (obstacle.Trajectory().trajectory_point().empty()) {
    return;
  }
  const double cos_angle_difference = std::cos(::math::NormalizeAngle(
      vehicle_state.heading() -
      obstacle.Trajectory().trajectory_point()[0].path_point().theta()));
  const bool is_on_conming =
      cos_angle_difference < -1.0 * oncoming_angle_threshold;
  obstacle.SetIsOncoming(is_on_conming);
}

void Frame::SetIsApproachingStop(Obstacle &obstacle) const {
  if (obstacle.Trajectory().trajectory_point().empty()) {
    return;
  }
  const bool is_approaching_stop =
      (obstacle.Trajectory().trajectory_point().size() > kTrajectoryMinPoints &&
       obstacle.Trajectory().trajectory_point().back().v() == 0.0);
  obstacle.SetIsApproachingStop(is_approaching_stop);
}

const std::vector<const Obstacle *> Frame::obstacles() const {
  return obstacles_.Items();
}

void Frame::AddObstacleRegionType(Obstacle &obstacle,
                                  const LocalRoute &current_local_route) {
  const auto &param = common::VehicleConfigHelper::GetConfig().vehicle_param();
  Box2d box = common::CalculateCenterBoxFromCGPose(
      planning_start_point_.path_point().x(),
      planning_start_point_.path_point().y(),
      planning_start_point_.path_point().theta(), param);
  SLBoundary adc_sl_boundary;
  if (!current_local_route.GetSLBoundary(box, adc_sl_boundary)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return;
  }
  const LocalRouteConfig local_route_config = current_local_route.GetConfig();
  double lane_left_width = local_route_config.default_local_route_width * 0.5;
  double lane_right_width = local_route_config.default_local_route_width * 0.5;
  current_local_route.GetLaneWidth(adc_sl_boundary.start_s(), lane_left_width,
                                   lane_right_width);
  const double current_lane_start_l = -lane_right_width;
  const double current_lane_end_l = lane_left_width;
  const double left_lane_start_l = current_lane_end_l;
  const double left_lane_end_l =
      left_lane_start_l + local_route_config.default_local_route_width;
  const double right_lane_start_l =
      current_lane_start_l - local_route_config.default_local_route_width;
  const double right_lane_end_l = current_lane_start_l;
  const double left_left_lane_start_l = left_lane_end_l;
  const double left_left_lane_end_l =
      left_lane_end_l + local_route_config.default_local_route_width;
  const double right_right_lane_end_l = right_lane_start_l;
  const double right_right_lane_start_l =
      right_right_lane_end_l - local_route_config.default_local_route_width;
  SLBoundary sl_boundary;
  if (!current_local_route.GetSLBoundary(obstacle.PerceptionBoundingBox(),
                                         sl_boundary)) {
    AERROR << " Failed to get sl boundary for obstacle:" << obstacle.Id();
    return;
  }
  const double center_l = (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
  const double s_buffer = 2.0;
  const double l_buffer = 0.2;
  const double delta_s =
      sl_boundary.end_s() - (adc_sl_boundary.start_s() + s_buffer);
  if ((center_l > current_lane_start_l && center_l < current_lane_end_l) ||
      (delta_s > 0.0 &&
       ((center_l < current_lane_start_l &&
         sl_boundary.end_l() > current_lane_start_l + l_buffer) ||
        (center_l > current_lane_end_l &&
         sl_boundary.start_l() < current_lane_end_l - l_buffer)))) {
    obstacle.SetPerceptionSlBoundary(sl_boundary);
    if (delta_s >= 0.0 && delta_s < local_route_config.obs_filter_front_end_s) {
      obstacle.SetRelativeRegion2LocalRoute(RelativeRegionType::FRONT);
    } else if (delta_s < 0.0 &&
               delta_s > local_route_config.obs_filter_rear_start_s) {
      obstacle.SetRelativeRegion2LocalRoute(RelativeRegionType::REAR);
    }
  } else if (center_l > left_lane_start_l && center_l < left_lane_end_l) {
    obstacle.SetPerceptionSlBoundary(sl_boundary);
    if (delta_s >= local_route_config.obs_filter_front_start_s &&
        delta_s < local_route_config.obs_filter_front_end_s) {
      obstacle.SetRelativeRegion2LocalRoute(RelativeRegionType::LEFT_FRONT);

    } else if (delta_s < local_route_config.obs_filter_front_start_s &&
               delta_s > local_route_config.obs_filter_rear_end_s) {
      obstacle.SetRelativeRegion2LocalRoute(RelativeRegionType::LEFT);

    } else if (delta_s <= local_route_config.obs_filter_rear_end_s &&
               delta_s > local_route_config.obs_filter_rear_start_s) {
      obstacle.SetRelativeRegion2LocalRoute(RelativeRegionType::LEFT_REAR);
    }
  } else if (center_l > right_lane_start_l && center_l < right_lane_end_l) {
    obstacle.SetPerceptionSlBoundary(sl_boundary);
    if (delta_s >= local_route_config.obs_filter_front_start_s &&
        delta_s < local_route_config.obs_filter_front_end_s) {
      obstacle.SetRelativeRegion2LocalRoute(RelativeRegionType::RIGHT_FRONT);
    } else if (delta_s < local_route_config.obs_filter_front_start_s &&
               delta_s > local_route_config.obs_filter_rear_end_s) {
      obstacle.SetRelativeRegion2LocalRoute(RelativeRegionType::RIGHT);

    } else if (delta_s <= local_route_config.obs_filter_rear_end_s &&
               delta_s > local_route_config.obs_filter_rear_start_s) {
      obstacle.SetRelativeRegion2LocalRoute(RelativeRegionType::RIGHT_REAR);
    }
  } else if (center_l > left_left_lane_start_l &&
             center_l < left_left_lane_end_l) {
    obstacle.SetPerceptionSlBoundary(sl_boundary);
    if (delta_s >= local_route_config.obs_filter_rear_end_s &&
        delta_s < local_route_config.obs_filter_front_end_s) {
      obstacle.SetRelativeRegion2LocalRoute(
          RelativeRegionType::LEFT_LEFT_FRONT);
    }
  } else if (center_l > right_right_lane_start_l &&
             center_l < right_right_lane_end_l) {
    obstacle.SetPerceptionSlBoundary(sl_boundary);
    if (delta_s >= local_route_config.obs_filter_rear_end_s &&
        delta_s < local_route_config.obs_filter_front_end_s) {
      obstacle.SetRelativeRegion2LocalRoute(
          RelativeRegionType::RIGHT_RIGHT_FRONT);
    }
  }
}

void Frame::UpdateLeftLocalRoute() {
  uint8_t index = 0;
  for (auto &local_route : local_routes_) {
    if (index == 0) {
      index++;
      continue;
    }
    if (local_route.Lanes().GetSegmentType() ==
        hdmap::RouteSegments::SegmentType::LeftSegment) {
      local_routes_.begin()->SetLeftRoute(&local_route);
      break;
    }
    index++;
  }
}

void Frame::UpdateRightLocalRoute() {
  uint8_t index = 0;
  for (auto &local_route : local_routes_) {
    if (index == 0) {
      index++;
      continue;
    }
    if (local_route.Lanes().GetSegmentType() ==
        hdmap::RouteSegments::SegmentType::RightSegment) {
      local_routes_.begin()->SetRightRoute(&local_route);
      break;
    }
    index++;
  }
}

}  // namespace planning
}  // namespace zark
