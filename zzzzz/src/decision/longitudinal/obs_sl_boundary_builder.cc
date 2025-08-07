/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file obs_sl_boundary_builder.cc
 **/

#include "apps/planning/src/decision/longitudinal/obs_sl_boundary_builder.h"

#include "polygon2d.h"

namespace zark {
namespace planning {

using ::math::Polygon2d;
using ::math::Vec2d;

std::unordered_map<const Obstacle*, SLTrajectory>
ObsSLBoundaryBuilder::BuildObsSLBoundaryMap(
    const Corridor& corridor, const std::vector<const Obstacle*>& obstacles) {
  std::unordered_map<const Obstacle*, SLTrajectory> obs_sl_boundary_map;
  constexpr int kMinNumPoints = 2;
  if (corridor.size() < kMinNumPoints) {
    AERROR << "Number of corridor points is too few.";
    return obs_sl_boundary_map;
  }
  for (auto obs : obstacles) {
    if (obs == nullptr) {
      AERROR << "Obstacle is nullptr.";
      continue;
    }
    SLTrajectory sl_trajectory = ComputeObstacleSLBoundary(corridor, obs);
    obs_sl_boundary_map[obs] = sl_trajectory;
  }
  return obs_sl_boundary_map;
}

SLTrajectory ObsSLBoundaryBuilder::ComputeObstacleSLBoundary(
    const Corridor& corridor, const Obstacle* obstacle) {
  const auto& obs_trajectory = obstacle->Trajectory();
  const int obs_traj_size = obs_trajectory.trajectory_point().size();
  constexpr double kVirtualFenceLength = 1.0;

  if (obs_traj_size == 0) {
    AERROR << "Obstacle trajectory size is zero.";
  }

  int delta_k = 1;  // time index increment of sampled obs traj points
  bool is_end_point_needed = false;
  auto ComputeSLBoundarySize = [&delta_k, &is_end_point_needed, &obs_trajectory,
                                &obstacle]() {    
    if (PlanningGflags::enable_reduce_obs_traj_sample_t &&
        obs_trajectory.trajectory_point().size() > 10 && !obstacle->IsApproachingStop()) {
      double dt_obs = obs_trajectory.trajectory_point()[1].relative_time() -
                      obs_trajectory.trajectory_point()[0].relative_time();
      double dt_obs_sample =
          obs_trajectory.trajectory_point()[10].relative_time() -
          obs_trajectory.trajectory_point()[0].relative_time();
      if (!obstacle->IsStatic()) {
        delta_k = std::ceil(dt_obs_sample / dt_obs);
        is_end_point_needed = true;
      }
      return static_cast<int>(std::ceil(static_cast<double>(
                 obs_trajectory.trajectory_point().size() / delta_k))) +
             1;
    } else {
      return static_cast<int>(obs_trajectory.trajectory_point().size());
    }
  };

  SLTrajectory sl_trajectory;
  sl_trajectory.reserve(ComputeSLBoundarySize());

  auto ComputeSLBoundary =
      [&corridor, &obstacle](const ::common::TrajectoryPoint& traj_pt) {
        double start_s = std::numeric_limits<double>::max();
        double end_s = -std::numeric_limits<double>::max();
        double start_l = std::numeric_limits<double>::max();
        double end_l = -std::numeric_limits<double>::max();
        if (obstacle->IsVirtual()) {
          ::common::SLPoint sl_point;
          corridor.XYToSL({traj_pt.path_point().x(), traj_pt.path_point().y()},
                          sl_point);
          start_s = std::fmin(start_s, sl_point.s());
          end_s = std::fmax(end_s, sl_point.s() + kVirtualFenceLength);
          start_l = 0.0;
          end_l = 0.0;
        } else {
          Polygon2d cur_polygon = obstacle->GetPolygonAtPoint(traj_pt);
          for (const auto& p : cur_polygon.points()) {
            ::common::SLPoint sl_point;
            corridor.XYToSL(p, sl_point);
            start_s = std::fmin(start_s, sl_point.s());
            end_s = std::fmax(end_s, sl_point.s());
            start_l = std::fmin(start_l, sl_point.l());
            end_l = std::fmax(end_l, sl_point.l());
          }
        }
        SLBoundary sl_boundary;
        sl_boundary.set_start_s(start_s);
        sl_boundary.set_end_s(end_s);
        sl_boundary.set_start_l(start_l);
        sl_boundary.set_end_l(end_l);
        sl_boundary.set_v(traj_pt.v());
        sl_boundary.set_t(traj_pt.relative_time());
        return sl_boundary;
      };

  for (int k = 0; k < obs_traj_size; k += delta_k) {
    sl_trajectory.emplace_back(
        ComputeSLBoundary(obs_trajectory.trajectory_point()[k]));
  }
  // Process the last trajectory point if not already included
  if (is_end_point_needed) {
    sl_trajectory.emplace_back(
        ComputeSLBoundary(obs_trajectory.trajectory_point().back()));
  }

  return sl_trajectory;
}

}  // namespace planning
}  // namespace zark
