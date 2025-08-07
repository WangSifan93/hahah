/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file: st_graph_builder.cc
 **/
#include "apps/planning/src/decision/longitudinal/st_graph_builder.h"

#include <algorithm>
#include <memory>
#include <queue>

#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "line_segment2d.h"
#include "linear_interpolation.h"

namespace zark {
namespace planning {

using ::common::ErrorCode;
using ::common::Status;
using ::math::Box2d;
using ::math::LineSegment2d;
using ::math::Vec2d;

STGraphBuilder::STGraphBuilder(
    const LongitudinalDeciderConfig::STGraphConfig& config)
    : config_(config) {}

Status STGraphBuilder::BuildSTGraph(
    const Corridor& corridor,
    const IndexedPtrList<std::string, const Obstacle*>& obstacle_map,
    const std::unordered_map<const Obstacle*, SLTrajectory>&
        obs_sl_boundary_map,
    std::vector<STBoundary>& st_graph,
    std::vector<std::pair<const Obstacle*, bool>>& lateral_obstacles,
    std::unordered_map<std::string, int>& st_boundary_counter_map) {
  constexpr int kNumMin = 2;
  if (corridor.size() < kNumMin) {
    const std::string msg = "Number of corridor points is too few.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (MapObstaclesToSTBoundaries(corridor, obstacle_map, obs_sl_boundary_map,
                                 st_graph, lateral_obstacles,
                                 st_boundary_counter_map)
          .code() == ErrorCode::PLANNING_ERROR) {
    const std::string msg = "Mapping obstacle failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

Status STGraphBuilder::MapObstaclesToSTBoundaries(
    const Corridor& corridor,
    const IndexedPtrList<std::string, const Obstacle*>& obstacle_map,
    const std::unordered_map<const Obstacle*, SLTrajectory>&
        obs_sl_boundary_map,
    std::vector<STBoundary>& st_graph,
    std::vector<std::pair<const Obstacle*, bool>>& lateral_obstacles,
    std::unordered_map<std::string, int>& st_boundary_counter_map) {
  st_graph.clear();
  lateral_obstacles.reserve(obstacle_map.Items().size());
  UpdateSTBoundaryCounterMap(obstacle_map, st_boundary_counter_map);
  for (const auto& obstacle_ptr : obstacle_map.Items()) {
    if (obstacle_ptr == nullptr) {
      const std::string msg = "Obstacle is nullptr.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Draw the obstacle's st-boundary.
    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
    bool is_lateral = false;
    bool is_left = false;
    const SLTrajectory sl_trajectory = obs_sl_boundary_map.at(obstacle_ptr);

    if (config_.use_obs_sl_for_st_graph && !obstacle_ptr->IsApproachingStop()
            ? ComputeObstacleSTBoundaryByObsSL(
                  *obstacle_ptr, corridor, sl_trajectory, lower_points,
                  upper_points, is_lateral, is_left)
            : ComputeObstacleSTBoundaryByObsOverlap(
                  *obstacle_ptr, corridor, sl_trajectory, lower_points,
                  upper_points, is_lateral, is_left)) {
      st_boundary_counter_map[obstacle_ptr->Id()]++;
    } else {
      st_boundary_counter_map[obstacle_ptr->Id()] = 0;
    }

    if (st_boundary_counter_map[obstacle_ptr->Id()] >=
        config_.st_boundary_counter_min) {
      st_graph.emplace_back(STBoundary::CreateInstanceAccurate(
          lower_points, upper_points, obstacle_ptr->Id()));
    }

    if (is_lateral) {
      lateral_obstacles.emplace_back(std::make_pair(obstacle_ptr, is_left));
    }
  }
  return Status::OK();
}

void STGraphBuilder::UpdateSTBoundaryCounterMap(
    const IndexedPtrList<std::string, const Obstacle*>& obstacle_map,
    std::unordered_map<std::string, int>& st_boundary_counter_map) {
  for (auto it = st_boundary_counter_map.begin();
       it != st_boundary_counter_map.end();) {
    const std::string id = it->first;
    if (obstacle_map.Find(id) == nullptr) {
      it = st_boundary_counter_map.erase(it);
    } else {
      ++it;
    }
  }
}

bool STGraphBuilder::ComputeObstacleSTBoundaryByObsOverlap(
    const Obstacle& obstacle, const Corridor& adc_path_points,
    const SLTrajectory& sl_trajectory, std::vector<STPoint>& lower_points,
    std::vector<STPoint>& upper_points, bool& is_lateral, bool& is_left) {
  lower_points.clear();
  upper_points.clear();
  const auto& obs_trajectory = obstacle.Trajectory();

  // Go through every occurrence of the obstacle at all timesteps, and
  // figure out the overlapping s-max and s-min one by one.
  if (sl_trajectory.size() != obs_trajectory.trajectory_point().size()) {
    AERROR << "SL boundary size is not equal to trajectory point size.";

    return false;
  }
  int count = 0;
  for (const auto& obs_traj_pt : obs_trajectory.trajectory_point()) {
    const Box2d& obs_box = obstacle.GetBoundingBox(obs_traj_pt);
    ADEBUG << obs_box.DebugString();
    std::pair<double, double> overlapping_s;
    bool is_overlapping = GetOverlappingS(
        adc_path_points, obs_box, config_.safety_l_buffer, &overlapping_s);
    if (is_overlapping) {
      ADEBUG << "Obstacle instance is overlapping with ADC path.";
      lower_points.emplace_back(overlapping_s.first,
                                obs_traj_pt.relative_time());
      upper_points.emplace_back(overlapping_s.second,
                                obs_traj_pt.relative_time());
    }
    if (!is_lateral && !is_overlapping) {
      DetermineLeftOrRightObstacle(adc_path_points, sl_trajectory[count],
                                   is_lateral, is_left);
    }
    count++;
  }
  if (lower_points.size() == 1) {
    lower_points.emplace_back(lower_points.front().s(),
                              lower_points.front().t() + 0.1);
    upper_points.emplace_back(upper_points.front().s(),
                              upper_points.front().t() + 0.1);
  }
  return (!lower_points.empty() && !upper_points.empty());
}

bool STGraphBuilder::ComputeObstacleSTBoundaryByObsSL(
    const Obstacle& obstacle, const Corridor& corridor,
    const SLTrajectory& sl_trajectory, std::vector<STPoint>& lower_points,
    std::vector<STPoint>& upper_points, bool& is_lateral, bool& is_left) {
  static constexpr double kDeltaSTol = 1.0e-6;
  lower_points.clear();
  upper_points.clear();
  const auto& obs_trajectory = obstacle.Trajectory();
  const int obs_traj_size = obs_trajectory.trajectory_point().size();
  const auto vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  int k_max = obs_trajectory.trajectory_point().size();

  auto SearchAndUpdateSTPoint = [&obs_trajectory, &sl_trajectory, &corridor,
                                 &upper_points, &lower_points, &vehicle_param,
                                 &is_left, &is_lateral, &k_max,
                                 this](const bool& is_forward,
                                       const bool& is_oncoming,
                                       const bool& is_static_obstacle) {
    const double back_edge_to_cg =
        vehicle_param.back_edge_to_center() + vehicle_param.rear_axle_to_cg();
    const double front_edge_to_cg = vehicle_param.length() - back_edge_to_cg;
    const int start =
        is_forward ? 0 : obs_trajectory.trajectory_point().size() - 1;
    const int end = k_max;
    const int step = is_forward ? 1 : -1;
    double t_end = 0.0;        // serach end time for oncoming obstacle
    bool is_operated = false;  // point is inserted
    for (int k = start; k != end; k += step) {
      const double t = obs_trajectory.trajectory_point()[k].relative_time();
      const SLBoundary& sl_boundary = sl_trajectory.EvaluateByT(t);

      if (!lower_points.empty()) {
        t_end = lower_points.front().t() + config_.delta_t_oncoming_obs;
        if (t > t_end && is_oncoming) break;
      }

      if (sl_boundary.start_s() > corridor.back().s ||
          sl_boundary.end_s() < corridor.front().s ||
          !IsFullyBlocking(corridor,
                           0.5 * (sl_boundary.start_s() + sl_boundary.end_s()),
                           sl_boundary.start_l(), sl_boundary.end_l())) {
        if (!is_lateral) {
          DetermineLeftOrRightObstacle(corridor, sl_boundary, is_lateral,
                                       is_left);
        }
        continue;
      }
      k_max = k;

      if (is_static_obstacle) {
        lower_points.emplace_back(sl_boundary.start_s() - front_edge_to_cg, t);
        upper_points.emplace_back(sl_boundary.end_s() + back_edge_to_cg, t);
        break;
      } else if (!is_operated) {
        lower_points.emplace_back(sl_boundary.start_s() - front_edge_to_cg, t);
        upper_points.emplace_back(sl_boundary.end_s() + back_edge_to_cg, t);
        is_operated = true;
      } else if (!lower_points.empty() &&
                 std::abs((sl_boundary.start_s() - front_edge_to_cg) -
                          lower_points.back().s()) < kDeltaSTol &&
                 std::abs((sl_boundary.end_s() + back_edge_to_cg) -
                          upper_points.back().s()) < kDeltaSTol) {
        lower_points.pop_back();
        upper_points.pop_back();
        lower_points.emplace_back(sl_boundary.start_s() - front_edge_to_cg, t);
        upper_points.emplace_back(sl_boundary.end_s() + back_edge_to_cg, t);
      } else {
        if (!is_oncoming) {
          break;
        }
      }

      if (!lower_points.empty()) {
        t_end = lower_points.front().t() + config_.delta_t_oncoming_obs;
      }
    }
  };

  if (PlanningGflags::enable_reduce_obs_traj_sample_t) {
    SearchAndUpdateSTPoint(true, obstacle.IsOncoming(), obstacle.IsStatic());
    if (!lower_points.empty() && !obstacle.IsOncoming()) {
      SearchAndUpdateSTPoint(false, false, obstacle.IsStatic());
    }
  } else {
    const double back_edge_to_cg =
        vehicle_param.back_edge_to_center() + vehicle_param.rear_axle_to_cg();
    const double front_edge_to_cg = vehicle_param.length() - back_edge_to_cg;
    if (sl_trajectory.size() != obs_trajectory.trajectory_point().size()) {
      AERROR << "SL boundary size is not equal to trajectory point size.";
    }
    for (int i = 0; i < obs_traj_size; ++i) {
      const SLBoundary sl_boundary = sl_trajectory[i];
      if (sl_boundary.start_s() > corridor.back().s ||
          sl_boundary.end_s() < corridor.front().s ||
          (!IsFullyBlocking(corridor,
                            0.5 * (sl_boundary.start_s() + sl_boundary.end_s()),
                            sl_boundary.start_l(), sl_boundary.end_l()))) {
        if (!is_lateral)
          DetermineLeftOrRightObstacle(corridor, sl_boundary, is_lateral,
                                       is_left);
        continue;
      }
      lower_points.emplace_back(
          sl_boundary.start_s() - front_edge_to_cg,
          obs_trajectory.trajectory_point()[i].relative_time());
      upper_points.emplace_back(
          sl_boundary.end_s() + back_edge_to_cg,
          obs_trajectory.trajectory_point()[i].relative_time());
    }
  }

  if (lower_points.size() == 1) {
    lower_points.clear();
    upper_points.clear();
  }
  return (!lower_points.empty() && !upper_points.empty());
}

bool STGraphBuilder::IsFullyBlocking(const Corridor& corridor, const double s,
                                     const double l_start, const double l_end) {
  const double ego_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width();
  const CorridorPoint pt = corridor.EvaluateByS(s);
  if (PlanningGflags::use_corridor_boundary_blocking_judgement) {
    if (l_start > pt.l_left || l_end < pt.l_right) {
      return false;
    }
  } else {
    if (pt.l_left - l_end > ego_width + config_.fully_blocking_buffer ||
        l_start - pt.l_right > ego_width + config_.fully_blocking_buffer) {
      return false;
    }
  }
  return true;
}

bool STGraphBuilder::GetOverlappingS(
    const Corridor& adc_path_points, const ::math::Box2d& obstacle_instance,
    const double adc_l_buffer, std::pair<double, double>* const overlapping_s) {
  // Locate the possible range to search in details.
  constexpr int kDeltaNum = 2;
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double back_edge_to_cg =
      vehicle_param.back_edge_to_center() + vehicle_param.rear_axle_to_cg();
  double front_edge_to_cg = vehicle_param.length() - back_edge_to_cg;
  int pt_before_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, front_edge_to_cg, true, 0,
      static_cast<int>(adc_path_points.size()) - kDeltaNum);
  ADEBUG << "The index before is " << pt_before_idx;
  int pt_after_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, back_edge_to_cg, false, 0,
      static_cast<int>(adc_path_points.size()) - kDeltaNum);
  ADEBUG << "The index after is " << pt_after_idx;
  if (pt_before_idx == static_cast<int>(adc_path_points.size()) - kDeltaNum) {
    return false;
  }
  if (pt_after_idx == 0) {
    return false;
  }

  if (pt_before_idx == -1) {
    pt_before_idx = 0;
  }
  if (pt_after_idx == -1) {
    pt_after_idx = static_cast<int>(adc_path_points.size()) - kDeltaNum;
  }
  if (pt_before_idx >= pt_after_idx) {
    return false;
  }

  // Detailed searching.
  bool has_overlapping = false;
  for (int i = pt_before_idx; i <= pt_after_idx; ++i) {
    ADEBUG << "At ADC path index = " << i << " :";
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      overlapping_s->first = adc_path_points[std::max(i - 1, 0)].s;
      has_overlapping = true;
      ADEBUG << "There is overlapping.";
      break;
    }
  }
  if (!has_overlapping) {
    return false;
  }
  for (int i = pt_after_idx; i >= pt_before_idx; --i) {
    ADEBUG << "At ADC path index = " << i << " :";
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      overlapping_s->second = adc_path_points[i + 1].s;
      ADEBUG << "There is overlapping.";
      break;
    }
  }
  return true;
}

int STGraphBuilder::GetSBoundingPathPointIndex(const Corridor& adc_path_points,
                                               const Box2d& obstacle_instance,
                                               const double s_thresh,
                                               const bool is_before,
                                               const int start_idx,
                                               const int end_idx) {
  if (start_idx == end_idx) {
    if (IsPathPointAwayFromObstacle(adc_path_points[start_idx],
                                    adc_path_points[start_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return start_idx;
    } else {
      return -1;
    }
  }

  const int mid_idx =
      is_before ? (start_idx + end_idx - 1) / 2 + 1 : (start_idx + end_idx) / 2;
  const bool is_away = IsPathPointAwayFromObstacle(
      adc_path_points[mid_idx], adc_path_points[mid_idx + 1], obstacle_instance,
      s_thresh, is_before);
  const int i_start = is_before ? (is_away ? mid_idx : start_idx)
                                : (is_away ? start_idx : mid_idx + 1);
  const int i_end = is_before ? (is_away ? end_idx : mid_idx - 1)
                              : (is_away ? mid_idx : end_idx);
  return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                    s_thresh, is_before, i_start, i_end);
}

bool STGraphBuilder::IsPathPointAwayFromObstacle(
    const CorridorPoint& path_point, const CorridorPoint& direction_point,
    const Box2d& obs_box, const double s_thresh, const bool is_before) {
  Vec2d path_pt(path_point.xy_ref.x(), path_point.xy_ref.y());
  Vec2d dir_pt(direction_point.xy_ref.x(), direction_point.xy_ref.y());
  LineSegment2d path_dir_lineseg(path_pt, dir_pt);
  LineSegment2d normal_line_seg(path_pt, path_dir_lineseg.rotate(M_PI_2));

  auto corner_points = obs_box.GetAllCorners();
  for (const auto& corner_pt : corner_points) {
    Vec2d normal_line_ft_pt;
    normal_line_seg.GetPerpendicularFoot(corner_pt, &normal_line_ft_pt);
    Vec2d path_dir_unit_vec = path_dir_lineseg.unit_direction();
    Vec2d perpendicular_vec = corner_pt - normal_line_ft_pt;
    double corner_pt_s_dist = path_dir_unit_vec.InnerProd(perpendicular_vec);
    if (is_before && corner_pt_s_dist < s_thresh) {
      return false;
    }
    if (!is_before && corner_pt_s_dist > -s_thresh) {
      return false;
    }
  }
  return true;
}

bool STGraphBuilder::IsADCOverlappingWithObstacle(
    const CorridorPoint& adc_path_point, const ::math::Box2d& obs_box,
    const double l_buffer) {
  // Convert reference point from center of rear axis to center of ADC.
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  Vec2d ego_center_map_frame(adc_path_point.xy_ref.x(),
                             adc_path_point.xy_ref.y());
  ego_center_map_frame.SelfRotate(adc_path_point.theta);
  ego_center_map_frame.set_x(adc_path_point.xy_ref.x());
  ego_center_map_frame.set_y(adc_path_point.xy_ref.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, adc_path_point.theta,
                vehicle_param.length(), vehicle_param.width() + l_buffer * 2);

  ADEBUG << "    ADC box is: " << adc_box.DebugString();
  ADEBUG << "    Obs box is: " << obs_box.DebugString();

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  return obs_box.HasOverlap(adc_box);
}

void STGraphBuilder::DetermineLeftOrRightObstacle(
    const Corridor& adc_path_points, const SLBoundary sl_boundary,
    bool& is_lateral, bool& is_left) {
  double obstalce_s = 0.5 * (sl_boundary.end_s() + sl_boundary.start_s());
  double lateral_left = 0.0;
  double lateral_right = 0.0;
  if (!GetLeftAndRightLbyS(adc_path_points, obstalce_s, true, &lateral_left) ||
      !GetLeftAndRightLbyS(adc_path_points, obstalce_s, false,
                           &lateral_right)) {
    return;
  }
  if (sl_boundary.start_l() > 0.0 &&
      sl_boundary.start_l() - lateral_left < config_.lateral_buffer) {
    is_lateral = true;
    is_left = true;
    return;
  }
  if (sl_boundary.start_l() < 0.0 &&
      lateral_right - sl_boundary.end_l() < config_.lateral_buffer) {
    is_lateral = true;
    is_left = false;
    return;
  }
}

bool STGraphBuilder::GetLeftAndRightLbyS(const Corridor& adc_path_points,
                                         const double obstacle_s,
                                         const bool is_left, double* lateral) {
  if (obstacle_s < adc_path_points.front().s ||
      obstacle_s > adc_path_points.back().s || adc_path_points.size() < 2) {
    return false;
  }
  size_t match_idx = 0;
  for (size_t j = 1; j < adc_path_points.size(); j++) {
    if (obstacle_s >= adc_path_points[j - 1].s &&
        obstacle_s < adc_path_points[j].s) {
      match_idx = j - 1;
      break;
    }
  }
  if (is_left) {
    *lateral =
        adc_path_points[match_idx].l_left +
        (obstacle_s - adc_path_points[match_idx].s) *
            (adc_path_points[match_idx + 1].l_left -
             adc_path_points[match_idx].l_left) /
            (adc_path_points[match_idx + 1].s - adc_path_points[match_idx].s);
  } else {
    *lateral =
        adc_path_points[match_idx].l_right +
        (obstacle_s - adc_path_points[match_idx].s) *
            (adc_path_points[match_idx + 1].l_right -
             adc_path_points[match_idx].l_right) /
            (adc_path_points[match_idx + 1].s - adc_path_points[match_idx].s);
  }
  return true;
}

}  // namespace planning
}  // namespace zark
