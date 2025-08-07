/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_decider.cc
 **/

#include "apps/planning/src/tasks/deciders/lateral_decider/lateral_decider.h"

#include "apps/planning/src/common/local_route/local_route.h"
#include "apps/planning/src/common/local_route/local_route_point.h"
#include "cartesian_frenet_conversion.h"

namespace zark {
namespace planning {

using ::math::CartesianFrenetConverter;

static constexpr double kEpsilon = 1e-6;
static constexpr double kKmPHToMPS = 3.6;

LateralDecider::LateralDecider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Decider(config, injector),
      config_(config.task_config().lateral_decider_config) {
  v_target_coeff_table_ = LookupTable(config_.corridor.v_target_coeff_table);
  a_lat_max_table_ = LookupTable(config_.corridor.a_lat_max_table);
  const double half_car_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  l_left_corr_ = half_car_width + config_.corridor.extra_half_width;
  l_right_corr_ = -half_car_width - config_.corridor.extra_half_width;
  frenet_trajectory_map_builder_ =
      std::make_unique<FrenentTrajectoryMapBuilder>(config_);
}

::common::Status LateralDecider::Execute(Frame *frame) {
  frame_ = frame;
  ::common::Status status;
  const LocalRoute *current_local_route = frame_->FindCurrentLocalRoute();
  const LocalRoute *target_local_route = frame_->GetTargetLocalRoute();

  const CorridorInfo *const corridor_info_prev =
      (CheckCorridorInfoPrevValid(*(injector_->frame_history())))
          ? injector_->frame_history()
                ->Latest()
                ->GetProposals()
                .front()
                .GetCorridorInfo()
          : nullptr;

  const double v_map = frame_->local_view()
                           .fct_output->fct_out_bus_accinfosts()
                           .fct_out_v_accactsetspd_sg() /
                       kKmPHToMPS;

  if (frame_ != nullptr && current_local_route != nullptr &&
      target_local_route != nullptr) {
    const CorridorPoint corridor_start_point =
        ComputeCorridorStartPoint(frame_->IsReplan() || frame_->IsLatReplan());
    frenet_trajectory_map_builder_->SetEgoRealtimeVelocity(
        injector_->vehicle_state()->linear_velocity());
    TrajectoryMap trajectory_map =
        frenet_trajectory_map_builder_->GenerateTrajectoryMap(
            frame_->GetMission(), corridor_start_point,
            frame_->PlanningStartPoint(), *current_local_route,
            *target_local_route, frame->obstacles(), v_map, corridor_info_prev);

    for (const auto &[corridor_type, context] : trajectory_map) {
      const auto &[trajectory, v_target] = context;
      CorridorInfo corridor_info = CreateCorridorInfo(
          *current_local_route, *target_local_route, corridor_start_point,
          v_target, corridor_type, trajectory);
      frame_->MutableCorridorInfos()->emplace_back(std::move(corridor_info));
    }
    status = Status::OK();
  } else {
    status = Status(::common::ErrorCode::PLANNING_ERROR, "frame is nullptr.");
  }
  return status;
}

CorridorPoint LateralDecider::ComputeCorridorStartPoint(const bool is_replan) {
  CorridorPoint corridor_start_point;
  if (!is_replan) {
    ::math::Vec2d xy_point(frame_->PlanningStartPoint().path_point().x(),
                           frame_->PlanningStartPoint().path_point().y());
    ::common::SLPoint sl_point;
    const Corridor &corridor_prev = injector_->frame_history()
                                        ->Latest()
                                        ->GetProposals()
                                        .front()
                                        .GetCorridorInfo()
                                        ->GetCorridor();

    corridor_prev.XYToSL(xy_point, sl_point);
    corridor_start_point = corridor_prev.EvaluateByS(sl_point.s());
    corridor_start_point.v = frame_->PlanningStartPoint().v();
    corridor_start_point.a = frame_->PlanningStartPoint().a();
  } else {
    corridor_start_point.xy_ref.set_x(
        frame_->PlanningStartPoint().path_point().x());
    corridor_start_point.xy_ref.set_y(
        frame_->PlanningStartPoint().path_point().y());
    corridor_start_point.theta =
        frame_->PlanningStartPoint().path_point().theta();
    corridor_start_point.kappa =
        frame_->PlanningStartPoint().path_point().kappa();
    corridor_start_point.v = frame_->PlanningStartPoint().v();
    corridor_start_point.a = frame_->PlanningStartPoint().a();
  }
  return corridor_start_point;
}

Corridor LateralDecider::ComputeCorridor(
    const ::common::FrenetPoint &init_frenet_point,
    const LocalRoute &local_route, const CartesianTrajectory &traj_cartesian,
    const double v_target, const int i_start_frenet_traj,
    int &idx_start_point) {
  std::vector<CorridorPoint> corridor_points;
  constexpr int kNumMin = 2;
  if (traj_cartesian.size() < kNumMin) {
    AERROR << "Number of frenet trajectory sampling points is "
           << traj_cartesian.size();
    Corridor corridor(corridor_points);
    return corridor;
  }

  auto ComputeNumberOfPointsInSegments =
      [this](const ::common::FrenetPoint &init_frenet_point,
             const double s_traj_end, const CartesianTrajectory &traj_cartesian,
             const int i_start_frenet_traj, const double v_map,
             const double v_target, int &num_points_rear,
             int &num_points_middle, int &num_points_front,
             double &dt_end_seg) {
        const double dt = this->config_.frenet_trajectory_sampler_lc.dt;
        dt_end_seg = std::ceil((this->config_.corridor.sample_dist_min /
                                std::max(kEpsilon, v_target)) /
                               std::max(kEpsilon, dt)) *
                     dt;
        num_points_rear =
            std::floor(std::min(this->config_.corridor.t_rear,
                                std::max(init_frenet_point.s[kIdxS], 0.0) /
                                    std::max(config_.corridor.v_min,
                                             init_frenet_point.s[kIdxSDot])) /
                       std::max(kEpsilon, dt_end_seg));

        num_points_middle = i_start_frenet_traj + traj_cartesian.size();

        const double t_front =
            std::max(0.0, std::min(this->config_.corridor.length_max,
                                   std::max(this->config_.corridor.length_min,
                                            this->config_.corridor.t_total *
                                                v_map)) -
                              (s_traj_end - init_frenet_point.s[kIdxS])) /
            std::max(kEpsilon, v_target);
        num_points_front = std::ceil(t_front / std::max(kEpsilon, dt_end_seg));
      };

  int num_points_rear;
  int num_points_middle;
  int num_points_front;
  double dt_end_seg;
  const double v_map = frame_->local_view()
                           .fct_output->fct_out_bus_accinfosts()
                           .fct_out_v_accactsetspd_sg() /
                       kKmPHToMPS;
  ::common::SLPoint sl_pt_end;  // SL of the end point of trajectory w.r.t.
                                // local route [m]
  local_route.XYToSL(
      ::math::Vec2d(traj_cartesian.back().x, traj_cartesian.back().y),
      sl_pt_end);
  ComputeNumberOfPointsInSegments(init_frenet_point, sl_pt_end.s(),
                                  traj_cartesian, i_start_frenet_traj, v_map,
                                  v_target, num_points_rear, num_points_middle,
                                  num_points_front, dt_end_seg);
  corridor_points.reserve(num_points_rear + num_points_middle +
                          num_points_front);
  idx_start_point = num_points_rear;

  // append rear segment
  const double length_rear =
      num_points_rear * dt_end_seg *
      std::max(init_frenet_point.s[kIdxSDot], config_.corridor.v_min);
  double s_start_local_route = init_frenet_point.s[kIdxS] - length_rear;
  double s_start_corridor = -length_rear;
  ::common::SLPoint sl_pt_start;  // SL of the start point of trajectory w.r.t.
                                  // local route [m]
  local_route.XYToSL(
      ::math::Vec2d(traj_cartesian.front().x, traj_cartesian.front().y),
      sl_pt_start);
  ComputeEndCorridorSegment(local_route, s_start_local_route, s_start_corridor,
                            sl_pt_start.l(), init_frenet_point.s[kIdxSDot],
                            dt_end_seg, num_points_rear, corridor_points);

  // append middle segment
  ComputeMiddleCorridorSegment(init_frenet_point, local_route, traj_cartesian,
                               i_start_frenet_traj, corridor_points);

  // append front segment
  s_start_local_route = sl_pt_end.s() + v_target * dt_end_seg;
  s_start_corridor = corridor_points.back().s + v_target * dt_end_seg;
  ComputeEndCorridorSegment(local_route, s_start_local_route, s_start_corridor,
                            sl_pt_end.l(), traj_cartesian.back().v, dt_end_seg,
                            num_points_front, corridor_points);
  Corridor corridor(corridor_points);

  for (CorridorPoint &corridor_point : corridor) {
    corridor.SLToXY(::common::SLPoint(corridor_point.s, corridor_point.l_left),
                    &corridor_point.xy_left);
    corridor.SLToXY(::common::SLPoint(corridor_point.s, corridor_point.l_right),
                    &corridor_point.xy_right);
  }
  return corridor;
}

void LateralDecider::ComputeEndCorridorSegment(
    const LocalRoute &local_route, const double s_start_local_route,
    const double s_start_corridor, const double l, const double v,
    const double dt, const int num_points,
    std::vector<CorridorPoint> &corridor_points) {
  const bool is_front_segment =
      s_start_corridor >= 0.0 && corridor_points.size() > 0;
  const double ds = std::max(v, config_.corridor.v_min) * dt;
  for (int i = 0; i < num_points; ++i) {
    CorridorPoint corridor_point;
    corridor_point.s_ref =
        (i == 0) ? s_start_local_route : (corridor_points.back().s_ref + ds);
    if (corridor_point.s_ref > local_route.GetEndS()) {
      AINFO << "CorridorPoint s_ref is out of local route range."
            << corridor_point.s_ref << " " << local_route.GetEndS();
      break;
    }
    corridor_point.l_ref = l;
    corridor_point.s =
        (i == 0) ? s_start_corridor : (corridor_points.back().s + ds);
    corridor_point.l = 0.0;
    corridor_point.l_left = l_left_corr_;
    corridor_point.l_right = l_right_corr_;
    CalcLLeftAndRightBasedOnCurb(local_route, corridor_point.s_ref,
                                 corridor_point.l_ref, corridor_point.l_left,
                                 corridor_point.l_right);
    LocalRoutePoint matched_point;
    local_route.SLToXY(
        ::common::SLPoint(corridor_point.s_ref, corridor_point.l_ref),
        corridor_point.xy_ref, matched_point);
    corridor_point.theta = matched_point.heading();
    corridor_point.kappa = matched_point.kappa();
    if (is_front_segment) {
      corridor_point.v = corridor_points.back().v;
      corridor_point.a = 0.0;
      corridor_point.t = corridor_points.back().t + dt;
    } else {
      corridor_point.v = v;
      corridor_point.a = 0.0;
      corridor_point.t = 0.0;
    }
    AssignCorridorPointType(local_route, corridor_point);
    corridor_points.emplace_back(corridor_point);
  }
}

void LateralDecider::ComputeMiddleCorridorSegment(
    const ::common::FrenetPoint &init_frenet_point,
    const LocalRoute &local_route, const CartesianTrajectory &traj_cartesian,
    const int i_start_frenet_traj,
    std::vector<CorridorPoint> &corridor_points) {
  auto SafeDivide = [](const double num, const double denom) {
    if (std::abs(denom) < kEpsilon) {
      if (denom > 0) {
        return num / kEpsilon;
      } else {
        return -num / kEpsilon;
      }
    } else {
      return num / denom;
    }
  };

  // segment before frenet sampling trajectory
  const double dt = config_.frenet_trajectory_sampler_lc.dt;
  const double ds = init_frenet_point.s[kIdxSDot] * dt;
  for (int i = 0; i < i_start_frenet_traj; ++i) {
    CorridorPoint corridor_point;
    corridor_point.s_ref = init_frenet_point.s[kIdxS] + ds * i;
    if (corridor_point.s_ref > local_route.GetEndS()) {
      break;
    }
    corridor_point.l_ref = init_frenet_point.l[kIdxL];
    corridor_point.l = 0.0;
    corridor_point.l_left = l_left_corr_;
    corridor_point.l_right = l_right_corr_;
    CalcLLeftAndRightBasedOnCurb(local_route, corridor_point.s_ref,
                                 corridor_point.l_ref, corridor_point.l_left,
                                 corridor_point.l_right);
    LocalRoutePoint matched_point;
    local_route.SLToXY(
        ::common::SLPoint(corridor_point.s_ref, corridor_point.l_ref),
        corridor_point.xy_ref, matched_point);
    corridor_point.s =
        (i > 0) ? corridor_points.back().s + corridor_point.xy_ref.DistanceTo(
                                                 corridor_points.back().xy_ref)
                : 0.0;
    corridor_point.theta = matched_point.heading();
    corridor_point.kappa =
        SafeDivide(matched_point.kappa(),
                   (1.0 - matched_point.kappa() * corridor_point.l_ref));
    corridor_point.v = init_frenet_point.s[kIdxSDot];
    corridor_point.a = 0.0;
    corridor_point.t = i * dt;
    AssignCorridorPointType(local_route, corridor_point);
    corridor_points.emplace_back(corridor_point);
  }

  // segment of frenet sampling trajectory
  const double t_start = i_start_frenet_traj * dt;
  for (int i = 0; i < static_cast<int>(traj_cartesian.size()); ++i) {
    CorridorPoint corridor_point;
    ::common::SLPoint pt_frenet;
    local_route.XYToSL(::math::Vec2d(traj_cartesian[i].x, traj_cartesian[i].y),
                       pt_frenet);
    corridor_point.s_ref = pt_frenet.s();
    if (corridor_point.s_ref > local_route.GetEndS()) {
      break;
    }
    corridor_point.l_ref = pt_frenet.l();
    corridor_point.l = 0.0;
    corridor_point.l_left = l_left_corr_;
    corridor_point.l_right = l_right_corr_;
    CalcLLeftAndRightBasedOnCurb(local_route, corridor_point.s_ref,
                                 corridor_point.l_ref, corridor_point.l_left,
                                 corridor_point.l_right);
    corridor_point.xy_ref.set_x(traj_cartesian[i].x);
    corridor_point.xy_ref.set_y(traj_cartesian[i].y);
    corridor_point.v = traj_cartesian[i].v;
    corridor_point.a = traj_cartesian[i].a;
    corridor_point.theta = traj_cartesian[i].theta;
    corridor_point.kappa = traj_cartesian[i].kappa;
    corridor_point.s =
        (i == 0 && i_start_frenet_traj == 0)
            ? 0.0
            : corridor_points.back().s + corridor_point.xy_ref.DistanceTo(
                                             corridor_points.back().xy_ref);
    corridor_point.t = t_start + traj_cartesian[i].t;
    AssignCorridorPointType(local_route, corridor_point);
    corridor_points.emplace_back(corridor_point);
  }
}

void LateralDecider::CalcLLeftAndRightBasedOnCurb(const LocalRoute &local_route,
                                                  const double s_ref,
                                                  const double l_ref,
                                                  double &l_left,
                                                  double &l_right) {
  double dist_to_left_curb;
  double dist_to_right_curb;
  if (!local_route.GetDistanceToCurb(s_ref, dist_to_left_curb,
                                     dist_to_right_curb)) {
    const double dist_to_curb = 3.5;
    dist_to_left_curb = dist_to_curb;
    dist_to_right_curb = dist_to_curb;
  }
  const auto &veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  l_left = std::min(veh_param.width() * 0.5 + config_.corridor.extra_half_width,
                    dist_to_left_curb - l_ref);
  l_right =
      -std::min(veh_param.width() * 0.5 + config_.corridor.extra_half_width,
                dist_to_right_curb + l_ref);
}

IndexedPtrList<std::string, const Obstacle *> LateralDecider::FilterObstacles(
    const ::common::FrenetPoint &init_frenet_point,
    std::vector<const Obstacle *> obstacles,
    const Mission::LaneChangeRequest lane_change_request,
    const CorridorInfo::Type type) {
  IndexedPtrList<std::string, const Obstacle *> obstacles_map;
  for (const Obstacle *obstacle : obstacles) {
    const auto drv_st = obstacle->GetDriveStatus();
    const auto region_type = obstacle->RelativeRegionToLocalRoute();
    const bool is_lc_corridor = type == CorridorInfo::Type::LANE_CHANGE;
    const bool is_llc_req =
        lane_change_request == Mission::LaneChangeRequest::LC_LEFT;
    const bool is_rlc_req =
        lane_change_request == Mission::LaneChangeRequest::LC_RIGHT;

    if (region_type == RelativeRegionType::FRONT ||
        region_type == RelativeRegionType::LEFT ||
        region_type == RelativeRegionType::RIGHT ||
        region_type == RelativeRegionType::LEFT_FRONT ||
        region_type == RelativeRegionType::RIGHT_FRONT) {
      if (!CheckObstacleNeedIgnore(*obstacle, init_frenet_point,
                                   type == CorridorInfo::Type::LANE_KEEP)) {
        obstacles_map.Add(obstacle->Id(), obstacle);
      }
    }
    // update when left lc and lc corridor
    if (is_llc_req && is_lc_corridor) {
      const bool is_lr = region_type == RelativeRegionType::LEFT_REAR;
      const bool is_llf = region_type == RelativeRegionType::LEFT_LEFT_FRONT;
      // check if the rear and the obstacle has llc intent
      bool is_r_llc = region_type == RelativeRegionType::REAR;
      is_r_llc &= (drv_st == Obstacle::DriveStatus::STATUS_LANE_LEFT_CHANGE);

      if (is_lr || is_llf || is_r_llc) {
        obstacles_map.Add(obstacle->Id(), obstacle);
      }
    }
    // update when right lc and lc corridor
    if (is_rlc_req && is_lc_corridor) {
      const bool is_rr = region_type == RelativeRegionType::RIGHT_REAR;
      const bool is_rrf = region_type == RelativeRegionType::RIGHT_RIGHT_FRONT;
      // check if the rear and the obstacle has rlc intent
      bool is_r_rlc = region_type == RelativeRegionType::REAR;
      is_r_rlc &= (drv_st == Obstacle::DriveStatus::STATUS_LANE_RIGHT_CHANGE);

      if (is_rr || is_rrf || is_r_rlc) {
        obstacles_map.Add(obstacle->Id(), obstacle);
      }
    }

    const bool need_add_rear_obs =
        (lane_change_request == Mission::LaneChangeRequest::LANE_KEEP &&
         frame_ != nullptr && frame_->IsNearIntersection());
    if (need_add_rear_obs) {
      if (region_type == RelativeRegionType::LEFT_REAR ||
          region_type == RelativeRegionType::RIGHT_REAR) {
        obstacles_map.Add(obstacle->Id(), obstacle);
      }
    }
  }
  return obstacles_map;
}

void LateralDecider::AssignCorridorPointType(
    const LocalRoute &local_route, CorridorPoint &corridor_point) const {
  const std::vector<planning::LocalRoute::LineBoundary> &lane_boundaries =
      local_route.GetLaneBoundary(corridor_point.s_ref, 0.0);

  if (!lane_boundaries.empty()) {
    auto SetCorridorPointType = [](const auto &boundary_type) {
      switch (boundary_type) {
        case zark::hdmap_new::LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_CURB:
          return CorridorPoint::Type::CURB;
          break;
        case zark::hdmap_new::
            LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SINGLE_SOLID_LINE:
        case zark::hdmap_new::
            LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DOUBLE_SOLID:
          return CorridorPoint::Type::SOLID_LINE;
          break;
        case zark::hdmap_new::
            LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LONG_DASHED_LINE:
        case zark::hdmap_new::
            LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SHORT_DASHED_LINE:
        case zark::hdmap_new::
            LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SINGLE_DASHED_LINE:
          return CorridorPoint::Type::DASHED_LINE;
          break;
        default:
          return CorridorPoint::Type::OTHERS;
          break;
      }
    };
    corridor_point.type_left =
        SetCorridorPointType(lane_boundaries[0].left_type);
    corridor_point.type_right =
        SetCorridorPointType(lane_boundaries[0].right_type);
  } else {
    corridor_point.type_left = CorridorPoint::Type::UNKNOWN;
    corridor_point.type_right = CorridorPoint::Type::UNKNOWN;
  }

  auto IsCurb = [this](const CorridorPoint &corridor_point, bool &left_is_curb,
                       bool &right_is_curb) {
    if (corridor_point.l_left < this->l_left_corr_) {
      left_is_curb = true;
    }
    if (corridor_point.l_right > this->l_right_corr_) {
      right_is_curb = true;
    }
  };
  bool left_is_curb = false;
  bool right_is_curb = false;
  IsCurb(corridor_point, left_is_curb, right_is_curb);
  if (left_is_curb) {
    corridor_point.type_left = CorridorPoint::Type::CURB;
  }
  if (right_is_curb) {
    corridor_point.type_right = CorridorPoint::Type::CURB;
  }
}

CorridorInfo LateralDecider::CreateCorridorInfo(
    const LocalRoute &current_local_route, const LocalRoute &target_local_route,
    const CorridorPoint &corridor_start_point, const double v_target,
    const CorridorInfo::Type corridor_type,
    const CartesianTrajectory &traj_cartesian) {
  const LocalRoute &local_route =
      (corridor_type == CorridorInfo::Type::LANE_CHANGE ||
       corridor_type == CorridorInfo::Type::STAGING)
          ? target_local_route
          : current_local_route;

  int idx_start_point = 0;
  int i_start_frenet_traj = 0;
  if (corridor_type == CorridorInfo::Type::STAGING) {
    i_start_frenet_traj = std::ceil(config_.corridor.t_staging /
                                    config_.frenet_trajectory_sampler_lc.dt);
  }

  const Corridor corridor = ComputeCorridor(
      local_route.ToFrenetFrame(corridor_start_point), local_route,
      traj_cartesian, v_target, i_start_frenet_traj, idx_start_point);

  CorridorInfo corridor_info(
      local_route, frame_->GetMission(), corridor_type, corridor,
      idx_start_point,
      FilterObstacles(local_route.ToFrenetFrame(corridor_start_point),
                      frame_->obstacles(), frame_->GetMission().lc_request,
                      corridor_type));

  corridor_info.SetInitFrenetPoint(
      corridor.ToFrenetFrame(frame_->PlanningStartPoint()));
  AddDestinationVirtualObstacle(corridor_info);

  return corridor_info;
}

void LateralDecider::AddDestinationVirtualObstacle(
    CorridorInfo &corridor_info) {
  const Corridor &corridor = corridor_info.GetCorridor();
  const LocalRoute &local_route = corridor_info.GetLocalRoute();
  const double s_dest = local_route.GetDestinationS();
  const std::string id = "destination";
  if (corridor.crbegin()->s_ref > s_dest) {
    Obstacle obs_destination = Obstacle::CreateStaticVirtualObstacle(
        local_route, s_dest, id, frame_->ObstacleTimeHorizon());
    corridor_info.MutableVirtualObstacles()->emplace_back(
        std::move(obs_destination));
    const Obstacle &obs = corridor_info.GetVirtualObstacles().back();
    corridor_info.MutableObstacleMap()->Add(obs.Id(), &obs);
  } else {
    AWARN << "destination is not in the corridor";
  }
}

bool LateralDecider::CheckObstacleNeedIgnore(
    const Obstacle &obstacle, const ::common::FrenetPoint &ego_state,
    const bool is_lane_keep) {
  const double kMinDeltaS = 5.0;
  const double s_ego = ego_state.s[kIdxS];
  const double s_obs = obstacle.PerceptionSLBoundary().start_s();
  const double delta_s = std::max(s_obs - s_ego, kMinDeltaS);
  const double v_ego = ego_state.s[kIdxSDot];
  const double v_obs = obstacle.speed();
  const double acceleration =
      (v_obs * v_obs - v_ego * v_ego) /
      (2 * delta_s);  // for obs we want to filter, accel would be negative
  const bool is_lane_keep_rear_obstacle =
      is_lane_keep && s_ego - obstacle.PerceptionSLBoundary().end_s() > 0.0 &&
      !frame_->IsNearIntersection();
  return (delta_s > config_.corridor.s_distance_obstacle_need_ignore &&
          acceleration > config_.corridor.a_need_ignore) ||
         is_lane_keep_rear_obstacle;
}

const bool LateralDecider::CheckCorridorInfoPrevValid(
    const FrameHistory &frame_history) const {
  return frame_history.Latest() != nullptr &&
         !frame_history.Latest()->GetProposals().empty() &&
         frame_history.Latest()->GetProposals().front().GetCorridorInfo() !=
             nullptr;
}

const bool LateralDecider::CheckCorridorPrevValid(
    const FrameHistory &frame_history) const {
  return CheckCorridorInfoPrevValid(frame_history) && !frame_history.Latest()
                                                           ->GetProposals()
                                                           .front()
                                                           .GetCorridorInfo()
                                                           ->GetCorridor()
                                                           .empty();
}

}  // namespace planning
}  // namespace zark
