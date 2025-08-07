/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file frenet_trajectory_map_builder.cc
 **/

#include "frenet_trajectory_map_builder.h"

namespace zark {
namespace planning {

FrenentTrajectoryMapBuilder::FrenentTrajectoryMapBuilder(
    const LateralDeciderConfig &config) {
  config_corridor_ = config.corridor;
  v_target_coeff_table_ = LookupTable(config_corridor_.v_target_coeff_table);
  a_lat_max_table_ = LookupTable(config_corridor_.a_lat_max_table);
  frenet_trajectory_sampler_lk_ = std::make_shared<FrenetTrajectorySampler>(
      FrenetTrajectorySampler(config.frenet_trajectory_sampler_lk));
  frenet_trajectory_sampler_lc_ = std::make_shared<FrenetTrajectorySampler>(
      FrenetTrajectorySampler(config.frenet_trajectory_sampler_lc));
  frenet_trajectory_sampler_staging_ =
      std::make_shared<FrenetTrajectorySampler>(
          FrenetTrajectorySampler(config.frenet_trajectory_sampler_lc));
  frenet_trajectory_sampler_nudge_ = std::make_shared<FrenetTrajectorySampler>(
      FrenetTrajectorySampler(config.frenet_trajectory_sampler_nudge));
  const uint8_t hysteresis_cycle_number =
      config_corridor_.hysteresis_cycle_number;
  hysteresis_nudge_ = std::make_shared<hysteresis::Hysteresis>(
      hysteresis::Hysteresis(hysteresis_cycle_number));
  hysteresis_nudge_->InitState(false);
  ego_realtime_velocity_ = 0.0;
}

TrajectoryMap FrenentTrajectoryMapBuilder::GenerateTrajectoryMap(
    const Mission &mission, const CorridorPoint &corridor_start_point,
    const TrajectoryPoint &planning_start_point,
    const LocalRoute &current_local_route, const LocalRoute &target_local_route,
    const std::vector<const Obstacle *> &obstacles, const double v_map,
    const CorridorInfo *const corridor_info_prev) {
  TrajectoryMap trajectory_map;
  FrenetTrajectory lane_keep_traj;
  trajectory_map[CorridorInfo::Type::LANE_KEEP] = CreateTrajectoryInfo(
      current_local_route, target_local_route, corridor_start_point, v_map,
      CorridorInfo::Type::LANE_KEEP, &lane_keep_traj);

  const bool is_lc_requested =
      (mission.lc_request == Mission::LaneChangeRequest::LC_LEFT ||
       mission.lc_request == Mission::LaneChangeRequest::LC_RIGHT);
  if (is_lc_requested) {
    trajectory_map[CorridorInfo::Type::LANE_CHANGE] = CreateTrajectoryInfo(
        current_local_route, target_local_route, corridor_start_point, v_map,
        CorridorInfo::Type::LANE_CHANGE);

    if (PlanningGflags::enable_staging_corridor && !mission.is_near_ramp &&
        corridor_info_prev != nullptr &&
        corridor_info_prev->GetType() != CorridorInfo::Type::LANE_CHANGE) {
      trajectory_map[CorridorInfo::Type::STAGING] = CreateTrajectoryInfo(
          current_local_route, target_local_route, corridor_start_point, v_map,
          CorridorInfo::Type::STAGING);
    }
  }

  if (PlanningGflags::enable_nudge_corridor && !is_lc_requested) {
    // TODO: move to a new class
    const double v_target = CalculateVTarget(
        current_local_route,
        std::min(v_map, CaculateSpeedLimitByCurvature(current_local_route)),
        CorridorInfo::Type::NUDGE);
    std::vector<FrenetTrajectory> nudge_trajectories =
        GenerateNudgeTrajectories(corridor_start_point, current_local_route,
                                  v_target);

    FilterNudgeTrajectories(planning_start_point, lane_keep_traj,
                            current_local_route, obstacles, nudge_trajectories,
                            corridor_info_prev);
    if (hysteresis_nudge_->GetCurrentState() && !nudge_trajectories.empty()) {
      trajectory_map[CorridorInfo::Type::NUDGE] =
          std::make_pair(ConvertFrenetTrajToCartesianTraj(
                             current_local_route, nudge_trajectories.at(0)),
                         v_target);
    }
  }
  return trajectory_map;
}

std::pair<CartesianTrajectory, double>
FrenentTrajectoryMapBuilder::CreateTrajectoryInfo(
    const LocalRoute &current_local_route, const LocalRoute &target_local_route,
    const CorridorPoint &corridor_start_point, const double v_map,
    const CorridorInfo::Type &type, FrenetTrajectory *traj_frenet) {
  const LocalRoute &local_route = (type == CorridorInfo::Type::LANE_CHANGE ||
                                   type == CorridorInfo::Type::STAGING)
                                      ? target_local_route
                                      : current_local_route;
  const double v_target = CalculateVTarget(
      local_route, std::min(v_map, CaculateSpeedLimitByCurvature(local_route)),
      type);
  double l_target = 0.0;
  FrenetTrajectory traj =
      GenerateTrajectory(local_route.ToFrenetFrame(corridor_start_point),
                         v_target, l_target, type);
  const CartesianTrajectory traj_cartesian =
      ConvertFrenetTrajToCartesianTraj(local_route, traj);
  if (traj_frenet != nullptr) {
    *traj_frenet = std::move(traj);
  }
  return std::make_pair(traj_cartesian, v_target);
}

void FrenentTrajectoryMapBuilder::DetermineInitAndTerminalState(
    const FrenetPoint &init_frenet_point, const double v_target,
    const double l_target, const CorridorInfo::Type type,
    std::array<double, FrenetTrajectorySampler::kNumStates> &s_init,
    std::array<double, FrenetTrajectorySampler::kNumStates> &l_init,
    std::array<double, FrenetTrajectorySampler::kNumStates> &s_terminal,
    std::array<double, FrenetTrajectorySampler::kNumStates> &l_terminal) {
  // initial state
  s_init[1] = std::max(config_corridor_.v_min, init_frenet_point.s[kIdxSDot]);
  s_init[2] = init_frenet_point.s[kIdxSDDot];

  if (type == CorridorInfo::Type::STAGING) {
    s_init[0] = init_frenet_point.s[kIdxS] +
                config_corridor_.t_staging * init_frenet_point.s[kIdxSDot];
    l_init[0] = init_frenet_point.l[kIdxL];
    l_init[1] = 0.0;
    l_init[2] = 0.0;
  } else if (type == CorridorInfo::Type::LANE_KEEP) {
    s_init[0] = init_frenet_point.s[kIdxS];
    l_init[0] = 0.0;
    l_init[1] = 0.0;
    l_init[2] = 0.0;
  } else {
    s_init[0] = init_frenet_point.s[kIdxS];
    const double l_prime = init_frenet_point.l[kIdxLDot];
    const double l_pprime = init_frenet_point.l[kIdxLDDot];
    const double sdot_pow_2 = s_init[1] * s_init[1];
    l_init[0] = init_frenet_point.l[kIdxL];
    l_init[1] = l_prime * s_init[1];
    l_init[2] = l_pprime * sdot_pow_2 + l_prime * s_init[2];
  }
  // terminal state
  s_terminal[0] = std::numeric_limits<double>::quiet_NaN();
  s_terminal[1] = v_target;
  s_terminal[2] = 0.0;
  l_terminal[0] = l_target;
  l_terminal[1] = 0.0;
  l_terminal[2] = 0.0;
}

FrenetTrajectory FrenentTrajectoryMapBuilder::GenerateTrajectory(
    const FrenetPoint &init_frenet_point, const double v_target,
    const double l_target, const CorridorInfo::Type type) {
  std::array<double, FrenetTrajectorySampler::kNumStates> s_init;
  std::array<double, FrenetTrajectorySampler::kNumStates> l_init;
  std::array<double, FrenetTrajectorySampler::kNumStates> s_terminal;
  std::array<double, FrenetTrajectorySampler::kNumStates> l_terminal;
  DetermineInitAndTerminalState(init_frenet_point, v_target, l_target, type,
                                s_init, l_init, s_terminal, l_terminal);
  switch (type) {
    case CorridorInfo::Type::NUDGE:
      return frenet_trajectory_sampler_nudge_->ComputeOptimalTrajectory(
          s_init, l_init, s_terminal, l_terminal);
      break;
    case CorridorInfo::Type::LANE_KEEP:
      return frenet_trajectory_sampler_lk_->ComputeOptimalTrajectory(
          s_init, l_init, s_terminal, l_terminal);
      break;
    case CorridorInfo::Type::LANE_CHANGE:
      return frenet_trajectory_sampler_lc_->ComputeOptimalTrajectory(
          s_init, l_init, s_terminal, l_terminal);
      break;
    case CorridorInfo::Type::STAGING:
      return frenet_trajectory_sampler_staging_->ComputeOptimalTrajectory(
          s_init, l_init, s_terminal, l_terminal);
      break;
    default:
      return frenet_trajectory_sampler_lk_->ComputeOptimalTrajectory(
          s_init, l_init, s_terminal, l_terminal);
      break;
  }
}

std::vector<FrenetTrajectory>
FrenentTrajectoryMapBuilder::GenerateNudgeTrajectories(
    const CorridorPoint &corridor_start_point, const LocalRoute &local_route,
    const double v_target) {
  std::vector<FrenetTrajectory> nudge_trajectories;

  const double s_ego = local_route.InitFrenetPoint().s[kIdxS];
  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!local_route.GetLaneWidth(s_ego, left_lane_width, right_lane_width)) {
    AERROR << "Failed to get lane width.";
  }
  const double lane_width = left_lane_width + right_lane_width;
  double l_target = 0.0;
  if (local_route.LeftRoute() != nullptr ||
      lane_width > config_corridor_.lane_width_thres) {
    for (int i = 0; i < config_corridor_.num_l_target; i++) {
      l_target = config_corridor_.delta_l_target * (i + 1);
      nudge_trajectories.emplace_back(
          GenerateTrajectory(local_route.ToFrenetFrame(corridor_start_point),
                             v_target, l_target, CorridorInfo::Type::NUDGE));
    }
  }
  if (local_route.RightRoute() != nullptr ||
      lane_width > config_corridor_.lane_width_thres) {
    for (int i = 0; i < config_corridor_.num_l_target; i++) {
      l_target = -1.0 * config_corridor_.delta_l_target * (i + 1);
      nudge_trajectories.emplace_back(
          GenerateTrajectory(local_route.ToFrenetFrame(corridor_start_point),
                             v_target, l_target, CorridorInfo::Type::NUDGE));
    }
  }
  return nudge_trajectories;
}

void FrenentTrajectoryMapBuilder::FilterNudgeTrajectories(
    const TrajectoryPoint &planning_start_point,
    const FrenetTrajectory &lane_keep_trajectory, const LocalRoute &local_route,
    const std::vector<const Obstacle *> &obstacles,
    std::vector<FrenetTrajectory> &nudge_trajectories,
    const CorridorInfo *const corridor_info_prev) {
  const std::vector<const Obstacle *> region_obstacles =
      GetObstacles(local_route, obstacles, CorridorInfo::Type::LANE_KEEP);
  double nudge_obstacle_velocity = 0.0;
  double ego_obstacle_min_delta_l = 3.0;
  const double lane_keep_travel_time = CalcTrajectoryTravelableTime(
      lane_keep_trajectory, local_route, region_obstacles,
      ego_obstacle_min_delta_l, &nudge_obstacle_velocity);
  FilterTrajectoriesByBoundaries(local_route, nudge_trajectories);
  FilterTrajectoriesByProgress(planning_start_point, local_route, obstacles,
                               lane_keep_travel_time, nudge_obstacle_velocity,
                               nudge_trajectories, corridor_info_prev);
  SortTrajectoriesBySimilarity(planning_start_point, nudge_trajectories,
                               corridor_info_prev);
  return;
}

void FrenentTrajectoryMapBuilder::FilterTrajectoriesByBoundaries(
    const LocalRoute &local_route,
    std::vector<FrenetTrajectory> &nudge_trajectories) {
  auto IsNudgeTrajectoryValid = [this](const LocalRoute &local_route,
                                       const FrenetTrajectory &traj_frenet) {
    size_t n_traj_pts = traj_frenet.size();
    const int kNumTrajPtsToCheck = 4;
    if (n_traj_pts < kNumTrajPtsToCheck) {
      return false;
    }
    size_t n_pts_to_skip = size_t(n_traj_pts / kNumTrajPtsToCheck);
    const bool is_left_nudge = (traj_frenet.back().l[kIdxL] > 0);
    for (size_t i = 0; i < n_traj_pts; i = i + n_pts_to_skip) {
      double lane_left_width = 1.65;
      double lane_right_width = 1.65;
      if (traj_frenet.at(i).s[kIdxS] > local_route.GetEndS()) {
        continue;
      }
      local_route.GetLaneWidth(traj_frenet.at(i).s[kIdxS], lane_left_width,
                               lane_right_width);
      if (is_left_nudge &&
          fabs(traj_frenet.at(i).l[kIdxL]) >= lane_left_width) {
        return false;
      }
      if (!is_left_nudge &&
          fabs(traj_frenet.at(i).l[kIdxL]) >= lane_right_width) {
        return false;
      }

      double road_left_width = 1.65;
      double road_right_width = 1.65;
      local_route.GetRoadWidth(traj_frenet.at(i).s[kIdxS], road_left_width,
                               road_right_width);
      const auto &veh_param =
          common::VehicleConfigHelper::GetConfig().vehicle_param();
      const double distance2left_road_boundary =
          road_left_width - fabs(traj_frenet.at(i).l[kIdxL]) -
          veh_param.width() / 2.0;
      const double distance2right_road_boundary =
          road_right_width - fabs(traj_frenet.at(i).l[kIdxL]) -
          veh_param.width() / 2.0;
      const double nudge_safe_buffer =
          this->config_corridor_.obs_collision_buffer_table.regular_car;
      if (is_left_nudge && distance2left_road_boundary < nudge_safe_buffer) {
        return false;
      }
      if (!is_left_nudge && distance2right_road_boundary < nudge_safe_buffer) {
        return false;
      }
    }
    return true;
  };

  for (auto iter = nudge_trajectories.begin();
       iter != nudge_trajectories.end();) {
    if (!IsNudgeTrajectoryValid(local_route, *iter)) {
      nudge_trajectories.erase(iter);
      continue;
    }
    ++iter;
  }
}

void FrenentTrajectoryMapBuilder::FilterTrajectoriesByProgress(
    const TrajectoryPoint &planning_start_point, const LocalRoute &local_route,
    const std::vector<const Obstacle *> &obstacles,
    const double lane_keep_travel_time, const double nudge_obstacle_velocity,
    std::vector<FrenetTrajectory> &nudge_trajectories,
    const CorridorInfo *const corridor_info_prev) {
  const double nudge_time_gap = 0.3;
  double l_target_prev = (corridor_info_prev != nullptr)
                             ? corridor_info_prev->GetCorridor().back().l_ref
                             : 0.0;
  const double kLTargetDiffThreshold = 0.1;
  bool is_nudge_progress_short_than_lane_keep = false;
  for (auto iter = nudge_trajectories.begin();
       iter != nudge_trajectories.end();) {
    const bool is_left = iter->back().l[kIdxL] > 0.0;
    const std::vector<const Obstacle *> region_obstacles = GetObstacles(
        local_route, obstacles, CorridorInfo::Type::NUDGE, is_left);
    double ego_obstacle_min_delta_l = 3.0;
    const double nudge_travel_time = CalcTrajectoryTravelableTime(
        *iter, local_route, region_obstacles, ego_obstacle_min_delta_l);
    if (hysteresis_nudge_->GetCurrentState() &&
        std::fabs(iter->back().l[kIdxL] - l_target_prev) <
            kLTargetDiffThreshold) {
      if (nudge_travel_time - nudge_time_gap < lane_keep_travel_time &&
          lane_keep_travel_time >= config_corridor_.t_obs_collision_check) {
        is_nudge_progress_short_than_lane_keep = true;
      }
      iter++;
      continue;
    }
    const double nudge_safe_buffer =
        (config_corridor_.obs_collision_buffer_table.large_car +
         config_corridor_.obs_collision_buffer_table.regular_car) /
        4.0;
    const bool is_nudge_safe =
        (lane_keep_travel_time < config_corridor_.t_obs_collision_check) &&
        (ego_obstacle_min_delta_l > nudge_safe_buffer) &&
        CheckNudgeConditionReady(local_route.InitFrenetPoint().s[kIdxSDot],
                                 nudge_obstacle_velocity);
    if (!is_nudge_safe) {
      nudge_trajectories.erase(iter);
      continue;
    }
    ++iter;
  }

  if (nudge_trajectories.size() == 1 &&
      is_nudge_progress_short_than_lane_keep) {
    hysteresis_nudge_->UpdateState(false);
    AINFO << " do not need nudge:" << l_target_prev;
    return;
  }
  if (!nudge_trajectories.empty()) {
    hysteresis_nudge_->UpdateState(true);
    AINFO << " need nudge";
  } else {
    hysteresis_nudge_->UpdateState(false);
    AINFO << "do not need nudge";
  }
}

void FrenentTrajectoryMapBuilder::SortTrajectoriesBySimilarity(
    const TrajectoryPoint &planning_start_point,
    std::vector<FrenetTrajectory> &nudge_trajectories,
    const CorridorInfo *const corridor_info_prev) {
  if (nudge_trajectories.empty()) {
    return;
  }
  if (corridor_info_prev == nullptr) {
    return;
  }

  double nudge_trajectory_cost = CaculateNudgeTrajectorySimilarity(
      planning_start_point, nudge_trajectories.at(0), corridor_info_prev);
  for (size_t i = 1; i < nudge_trajectories.size(); i++) {
    double trajectory_cost = CaculateNudgeTrajectorySimilarity(
        planning_start_point, nudge_trajectories.at(i), corridor_info_prev);
    if (trajectory_cost < nudge_trajectory_cost) {
      nudge_trajectory_cost = trajectory_cost;
      std::swap(nudge_trajectories[0], nudge_trajectories[i]);
    }
  }
}

double FrenentTrajectoryMapBuilder::CaculateNudgeTrajectorySimilarity(
    const TrajectoryPoint &planning_start_point,
    const FrenetTrajectory &traj_frenet,
    const CorridorInfo *const corridor_info_prev) {
  double delta_s = planning_start_point.v() * planning_start_point.dt_prev();
  std::vector<double> l_diff;
  for (size_t i = 0; i < traj_frenet.size(); i++) {
    CorridorPoint pre_point = corridor_info_prev->GetCorridor().EvaluateByS(
        traj_frenet.at(i).s[0] + delta_s);
    l_diff.emplace_back(fabs(traj_frenet.at(i).l[0] - pre_point.l_ref));
  }
  if (l_diff.empty()) {
    return 0.0;
  }

  return std::accumulate(l_diff.begin(), l_diff.end(), 0.0) / l_diff.size();
}

CartesianTrajectory
FrenentTrajectoryMapBuilder::ConvertFrenetTrajToCartesianTraj(
    const LocalRoute &local_route, const FrenetTrajectory &traj_frenet) {
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

  CartesianTrajectory traj_cartesian;
  for (auto point : traj_frenet) {
    CartesianPoint cartesian_point;
    cartesian_point.t = point.t;
    const double l = point.l[kIdxL];
    const double lprime = SafeDivide(point.l[kIdxLDot], point.s[kIdxSDot]);
    const double lpprime =
        SafeDivide((point.l[kIdxLDDot] - lprime * point.s[kIdxSDDot]),
                   (point.s[kIdxSDot] * point.s[kIdxSDot]));
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double v = 0.0;
    double a = 0.0;
    std::array<double, FrenetTrajectorySampler::kNumStates> d_condition{
        l, lprime, lpprime};
    const LocalRoutePoint matched_point =
        local_route.GetLocalRoutePoint(point.s[kIdxS]);
    CartesianFrenetConverter::frenet_to_cartesian(
        point.s[kIdxS], matched_point.x(), matched_point.y(),
        matched_point.heading(), matched_point.kappa(), matched_point.dkappa(),
        point.s, d_condition, &x, &y, &theta, &kappa, &v, &a);
    cartesian_point.x = x;
    cartesian_point.y = y;
    cartesian_point.v = v;
    cartesian_point.a = a;
    cartesian_point.theta = theta;
    cartesian_point.kappa = kappa;
    traj_cartesian.emplace_back(cartesian_point);
  }
  return traj_cartesian;
}

const std::vector<const Obstacle *> FrenentTrajectoryMapBuilder::GetObstacles(
    const LocalRoute &local_route,
    const std::vector<const Obstacle *> &obstacles,
    const CorridorInfo::Type type, const bool is_left) {
  const double s_ego = local_route.InitFrenetPoint().s[kIdxS];
  std::string obj_id = "none";
  std::vector<const Obstacle *> region_obstacles;
  for (const Obstacle *obstacle : obstacles) {
    RelativeRegionType obstacle_region_type =
        obstacle->RelativeRegionToLocalRoute();
    if (obstacle_region_type == RelativeRegionType::FRONT ||
        obstacle_region_type == RelativeRegionType::LEFT_FRONT ||
        obstacle_region_type == RelativeRegionType::RIGHT_FRONT ||
        obstacle_region_type == RelativeRegionType::LEFT ||
        obstacle_region_type == RelativeRegionType::RIGHT) {
      const double distance = obstacle->PerceptionSLBoundary().end_s() - s_ego;
      const double safe_s_limit = -5.0;
      if (distance > safe_s_limit) {
        region_obstacles.emplace_back(obstacle);
      }
    }
    if (type == CorridorInfo::Type::NUDGE && is_left &&
        obstacle_region_type == RelativeRegionType::LEFT_REAR) {
      region_obstacles.emplace_back(obstacle);
    }
    if (type == CorridorInfo::Type::NUDGE && !is_left &&
        obstacle_region_type == RelativeRegionType::RIGHT_REAR) {
      region_obstacles.emplace_back(obstacle);
    }
  }
  auto compare = [](const Obstacle *A, const Obstacle *B) {
    return A->PerceptionSLBoundary().start_s() <
           B->PerceptionSLBoundary().start_s();
  };
  const size_t kNumObsMin = 2;
  if (region_obstacles.size() > kNumObsMin) {
    std::sort(region_obstacles.begin(), region_obstacles.end(), compare);
  }
  return region_obstacles;
}

double FrenentTrajectoryMapBuilder::CalcTrajectoryTravelableTime(
    const FrenetTrajectory &traj_frenet, const LocalRoute &local_route,
    const std::vector<const Obstacle *> &obstacles,
    double &ego_obstacle_min_delta_l, double *v_overlap_obs) {
  if (traj_frenet.empty()) {
    AINFO << "trajectory is empty";
    return 0.0;
  }
  double travel_time = traj_frenet.back().t;
  if (obstacles.empty()) {
    AINFO << "no obstacle in corridor";
    return travel_time;
  }
  const auto &veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  for (size_t i = 0; i < obstacles.size(); i++) {
    if (obstacles.at(i) != nullptr) {
      if (CheckNudgeObstacleNeedIgnore(obstacles.at(i), traj_frenet.at(0))) {
        continue;
      }
      const double large_car_width_lower = 2.1;
      const bool is_large_car =
          (obstacles.at(i)->Perception().width() > large_car_width_lower);
      double delta_l_target =
          std::min(fabs(traj_frenet.back().l[kIdxL] -
                        obstacles.at(i)->PerceptionSLBoundary().start_l()),
                   fabs(traj_frenet.back().l[kIdxL] -
                        obstacles.at(i)->PerceptionSLBoundary().end_l())) -
          veh_param.width() / 2.0;
      const auto &start_traj_point = obstacles.at(i)->GetPointAtTime(0.0);
      const Box2d &obstacle_start_box =
          obstacles.at(i)->GetBoundingBox(start_traj_point);
      for (const auto &point : traj_frenet) {
        const auto &traj_point = obstacles.at(i)->GetPointAtTime(point.t);
        const Box2d &obstacle_box = obstacles.at(i)->GetBoundingBox(traj_point);
        ::math::Vec2d xy_point;
        local_route.SLToXY(SLPoint(point.s[kIdxS], point.l[kIdxL]), xy_point);
        const LocalRoutePoint local_route_point =
            local_route.GetLocalRoutePoint(point.s[kIdxS]);
        const double heading = local_route_point.heading() +
                               atan2(point.l[kIdxLDot], point.s[kIdxSDot]);
        const double nudge_speed_lower = 7.5;
        const double obs_collision_buffer =
            (is_large_car && (ego_realtime_velocity_ > nudge_speed_lower))
                ? config_corridor_.obs_collision_buffer_table.large_car
                : config_corridor_.obs_collision_buffer_table.regular_car;
        const Box2d ego_box(xy_point, heading, veh_param.length(),
                            veh_param.width() + obs_collision_buffer);

        const double safe_buffer = 10.0;
        const bool ego_near_obstacle =
            ((traj_frenet.at(0).s[kIdxS] + veh_param.front_edge_to_center()) >
             (obstacles.at(i)->PerceptionSLBoundary().start_s() -
              safe_buffer)) &&
            ((traj_frenet.at(0).s[kIdxS] - veh_param.back_edge_to_center()) <
             (obstacles.at(i)->PerceptionSLBoundary().end_s() +
              0.5 * safe_buffer));

        if (point.t > config_corridor_.t_obs_collision_check) {
          if (point.t < travel_time) {
            travel_time = point.t;
            ego_obstacle_min_delta_l =
                std::min(ego_obstacle_min_delta_l, delta_l_target);
            const double near_large_car_travel_time = 2.0;
            if (ego_near_obstacle && is_large_car &&
                ego_realtime_velocity_ > nudge_speed_lower &&
                delta_l_target <
                    config_corridor_.obs_collision_buffer_table.large_car /
                        2.0) {
              travel_time = near_large_car_travel_time;
            }
            if (v_overlap_obs != nullptr) {
              *v_overlap_obs =
                  hypot(obstacles.at(i)->Perception().velocity().x(),
                        obstacles.at(i)->Perception().velocity().y());
            }
            break;
          }
        }

        if ((ego_box.HasOverlap(obstacle_box) ||
             ego_box.HasOverlap(obstacle_start_box)) &&
            (traj_frenet.at(0).s[kIdxSDot] > start_traj_point.v() ||
             (ego_near_obstacle && is_large_car))) {
          double temp_travel_time = point.t;
          const double delta_v = point.s[kIdxSDot] - start_traj_point.v();
          const double delta_v_lower = 1.0;
          if (ego_box.HasOverlap(obstacle_start_box) &&
              delta_v > delta_v_lower) {
            temp_travel_time =
                ego_box.HasOverlap(obstacle_box)
                    ? std::min((point.s[kIdxS] - traj_frenet.at(0).s[kIdxS]) /
                                   delta_v,
                               temp_travel_time)
                    : (point.s[kIdxS] - traj_frenet.at(0).s[kIdxS]) / delta_v;
          }
          if (temp_travel_time < travel_time) {
            travel_time = temp_travel_time;
            ego_obstacle_min_delta_l =
                std::min(ego_obstacle_min_delta_l, delta_l_target);
            if (v_overlap_obs != nullptr) {
              *v_overlap_obs =
                  hypot(obstacles.at(i)->Perception().velocity().x(),
                        obstacles.at(i)->Perception().velocity().y());
            }
          }
        }
      }
    }
  }
  return travel_time;
}

double FrenentTrajectoryMapBuilder::CalculateVTarget(
    const LocalRoute &local_route, const double v_map,
    const CorridorInfo::Type &type) {
  const double v_ego = local_route.InitFrenetPoint().s[kIdxSDot];
  const double coeff = v_target_coeff_table_.Evaluate(v_ego);
  const double a_comfort = 1.2;
  double t_total;
  switch (type) {
    case CorridorInfo::Type::NUDGE:
      t_total = frenet_trajectory_sampler_nudge_->SampleConfig().t_total;
      break;
    case CorridorInfo::Type::LANE_KEEP:
      t_total = frenet_trajectory_sampler_lk_->SampleConfig().t_total;
      break;
    case CorridorInfo::Type::LANE_CHANGE:
      t_total = frenet_trajectory_sampler_lc_->SampleConfig().t_total;
      break;
    case CorridorInfo::Type::STAGING:
      t_total = frenet_trajectory_sampler_staging_->SampleConfig().t_total;
      break;
    default:
      t_total = frenet_trajectory_sampler_lk_->SampleConfig().t_total;
      break;
  }
  const double v_max = std::min(v_map, v_ego + a_comfort * t_total);
  return std::max(config_corridor_.v_min,
                  coeff * v_ego + (1.0 - coeff) * v_max);
}

bool FrenentTrajectoryMapBuilder::CheckNudgeConditionReady(
    const double v_ego, const double v_obstacle) {
  return v_obstacle < config_corridor_.v_obs_min_to_enable_nudge ||
         v_ego > config_corridor_.v_relative_min_to_enable_nudge;
}

double FrenentTrajectoryMapBuilder::CaculateSpeedLimitByCurvature(
    const LocalRoute &local_route) {
  const double s_start = local_route.InitFrenetPoint().s[kIdxS];
  const double v_ego = local_route.InitFrenetPoint().s[kIdxSDot];
  const double s_end =
      s_start + config_corridor_.t_forward_by_curvature_spd_limit * v_ego;
  double kappa_max = std::numeric_limits<double>::min();
  const double kappa_check_number = 10.0;
  const double step = (s_end - s_start) / kappa_check_number;
  for (double s = s_start; s < s_end; s += step) {
    const LocalRoutePoint matched_point = local_route.GetLocalRoutePoint(s);
    kappa_max = std::max(fabs(matched_point.kappa()), kappa_max);
  }
  return std::sqrt(a_lat_max_table_.Evaluate(v_ego) /
                   std::max(kappa_max, kEpsilon));
}

std::shared_ptr<FrenetTrajectorySampler>
FrenentTrajectoryMapBuilder::GetFrenetTrajectorySamplerLK() const {
  return frenet_trajectory_sampler_lk_;
}

std::shared_ptr<FrenetTrajectorySampler>
FrenentTrajectoryMapBuilder::GetFrenetTrajectorySamplerLC() const {
  return frenet_trajectory_sampler_lc_;
}

std::shared_ptr<FrenetTrajectorySampler>
FrenentTrajectoryMapBuilder::GetFrenetTrajectorySamplerNudge() const {
  return frenet_trajectory_sampler_nudge_;
}

bool FrenentTrajectoryMapBuilder::CheckNudgeObstacleNeedIgnore(
    const Obstacle *obstacle, const FrenetPoint ego_position) {
  const double obs_center_l = (obstacle->PerceptionSLBoundary().start_l() +
                               obstacle->PerceptionSLBoundary().end_l()) /
                              2.0;
  const double current_lane_l_limit = 1.0;
  if (fabs(obs_center_l) < current_lane_l_limit) {
    return true;
  }
  const double delta_v =
      ego_realtime_velocity_ - obstacle->GetPointAtTime(0.0).v();
  const double delta_s =
      obstacle->PerceptionSLBoundary().start_s() > ego_position.s[kIdxS]
          ? obstacle->PerceptionSLBoundary().start_s() - ego_position.s[kIdxS]
          : obstacle->PerceptionSLBoundary().end_s() - ego_position.s[kIdxS];
  const double v_limit = config_corridor_.v_relative_min_to_enable_nudge;
  const double s_limit_upper = 30.0;
  const double s_limit_lower = -20.0;
  if ((delta_v < v_limit && delta_s > s_limit_upper) ||
      (delta_s < s_limit_lower)) {
    return true;
  }

  const double t_max = 8.0;
  if (delta_s > s_limit_upper) {
    return (delta_s / delta_v) > t_max;
  }
  return false;
}

}  // namespace planning
}  // namespace zark
