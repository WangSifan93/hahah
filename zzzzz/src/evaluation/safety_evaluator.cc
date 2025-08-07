/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file safety_evaluator.cc
 **/

#include "apps/planning/src/evaluation/safety_evaluator.h"

#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/common/conversion.h"
#include "apps/planning/src/common/log.h"
#include "apps/planning/src/evaluation/const_values.h"

namespace zark {
namespace planning {

namespace {
bool IsFrontTrafficInSTProposal(const std::string& id,
                                const STProposal& st_graph) {
  for (const auto& [slb, is_front, is_filtered] : st_graph) {
    if (slb->id() == id) {
      if (is_front == true && is_filtered == false) {
        return true;
      }
      return false;
    }
  }

  return false;
}

bool IsBackTrafficInSTProposal(const std::string& id,
                               const STProposal& st_graph) {
  for (const auto& [slb, is_front, is_filtered] : st_graph) {
    if (slb->id() == id) {
      if (is_front == false && is_filtered == false) {
        return true;
      }
      return false;
    }
  }

  return false;
}

}  // namespace

using ::math::Box2d;
using ::math::Polygon2d;
using ::math::Vec2d;

SafetyEvaluator::SafetyEvaluator(EvaluationDeciderConfig::Safety config)
    : config_(config),
      safety_cost_table_(config_.safety_cost_table),
      collision_time_weight_table_(config_.collision_time_weight_table),
      safety_lon_cost_table_(config_.safety_lon_cost_table),
      safety_lat_cost_table_(config_.safety_lat_cost_table),
      lc_time_table_(config_.lc_safety.safety_time_table) {}

Costs SafetyEvaluator::Evaluate(const Proposal& proposal,
                                const std::vector<const Obstacle*>& obstacles) {
  Costs safety_costs;
  EvaluateLonLatCollisionCost(proposal, obstacles, safety_costs);
  EvaluateSlackCost(proposal, safety_costs);
  // TODO(yiping): remove this temporary cost when lc safety cost is properly
  // done.
  EvaluateLCDangerousCost(proposal, obstacles, safety_costs);

  return safety_costs;
}

void SafetyEvaluator::EvaluateSlackCost(const Proposal& proposal,
                                        Costs& costs) {
  const auto& lat_mpc = proposal.GetLatMPCData();
  const auto& lon_mpc = proposal.GetLonMPCData();
  const auto& w_lat = config_.weight_lat_slack;
  const auto& w_lon = config_.weight_lon_slack;
  if (static_cast<int>(w_lat.weight_x.size()) == lat_mpc.n_x &&
      static_cast<int>(w_lat.weight_u.size()) == lat_mpc.n_u &&
      static_cast<int>(w_lat.weight_u_dot.size()) == lat_mpc.n_u_dot &&
      static_cast<int>(w_lon.weight_x.size()) == lon_mpc.n_x &&
      static_cast<int>(w_lon.weight_u.size()) == lon_mpc.n_u &&
      static_cast<int>(w_lon.weight_u_dot.size()) == lon_mpc.n_u_dot) {
    SlackCost(lat_mpc.slacks.stiff_min, w_lat, collision_time_weight_table_,
              lat_mpc.names, costs);
    SlackCost(lat_mpc.slacks.stiff_max, w_lat, collision_time_weight_table_,
              lat_mpc.names, costs);
    SlackCost(lon_mpc.slacks.stiff_min, w_lon, collision_time_weight_table_,
              lon_mpc.names, costs);
    SlackCost(lon_mpc.slacks.stiff_max, w_lon, collision_time_weight_table_,
              lon_mpc.names, costs);
  }
}

void SafetyEvaluator::EvaluateLonLatCollisionCost(
    const Proposal& proposal, const std::vector<const Obstacle*>& obstacles,
    Costs& costs) {
  // debug infos
  if (config_.enable_debug_info) {
    std::string obs_ids{};
    for (const auto obstacle : obstacles) {
      const auto ort = obstacle->RelativeRegionToLocalRoute();
      obs_ids += std::string("id: ") + obstacle->Id() +
                 ", ort: " + std::to_string(static_cast<int>(ort)) + ". ";
    }
    costs.emplace_back(
        std::make_pair(0.001, std::string("obstacles: ") + obs_ids));
  }

  const common::VehicleParam& ego_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const DiscretizedTrajectory& trajectory = proposal.GetTrajectory();
  const auto c_type = proposal.GetCorridorInfo()->GetType();
  double max_cost_time{0.0};
  double max_collision_cost{0.0};
  std::pair<double, double> max_cost_sl;
  std::string max_cost_obs_id{};
  for (std::int32_t k = 0; k < trajectory.size(); ++k) {
    const ::common::TrajectoryPoint& trajectory_point = trajectory.at(k);
    const ::math::Box2d& adc_box = common::CalculateCenterBoxFromCGPose(
        trajectory_point.path_point().x(), trajectory_point.path_point().y(),
        trajectory_point.path_point().theta(), ego_param);
    for (const Obstacle* obstacle : obstacles) {
      EvaluateCollisionCostObstacle(
          c_type, trajectory, trajectory_point, adc_box, *obstacle, ego_param,
          max_collision_cost, max_cost_sl, max_cost_time, max_cost_obs_id);
    }  // obstacles loop
  }    // trajectory points loop

  if (max_collision_cost > kCostEpsilon) {
    const std::string reason =
        "relative time: " + std::to_string(max_cost_time) +
        "s. s/l distance to obtacle " + max_cost_obs_id + ": " +
        std::to_string(max_cost_sl.first) + "/" +
        std::to_string(max_cost_sl.second);
    costs.emplace_back(std::make_pair(max_collision_cost, reason));
  }
}

void SafetyEvaluator::EvaluateLCDangerousCost(
    const Proposal& proposal, const std::vector<const Obstacle*>& obstacles,
    Costs& costs) {
  if (!config_.lc_safety.enable) {
    return;
  }

  const auto& e_info = common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto e_l_half = e_info.length() * 0.5;
  const auto& trajectory = proposal.GetTrajectory();
  const auto& c_info = *proposal.GetCorridorInfo();
  const auto& mission = c_info.GetMission();
  const bool is_llc = Mission::LaneChangeRequest::LC_LEFT == mission.lc_request;
  const auto& lc_conf = config_.lc_safety;
  // only focus on lane change
  if (c_info.GetType() != CorridorInfo::Type::LANE_CHANGE) {
    return;
  }

  auto dangerous_cost = 0.0;
  std::string dangerous_reason{};
  const auto& ego_pt = trajectory.EvaluateByT(0.0);
  for (const auto obstacle : obstacles) {
    const auto ort = obstacle->RelativeRegionToLocalRoute();
    const auto side_type =
        is_llc ? RelativeRegionType::LEFT : RelativeRegionType::RIGHT;
    const auto side_r_type =
        is_llc ? RelativeRegionType::LEFT_REAR : RelativeRegionType::RIGHT_REAR;
    // only consider the side/rear_side and back traffic in st graph
    if ((ort != side_r_type && ort != side_type)) {
      continue;
    }

    // ignore the front traffic in st graph
    const auto is_front_st =
        IsFrontTrafficInSTProposal(obstacle->Id(), proposal.GetSTProposal());
    if (lc_conf.filter_front_obs_in_st && is_front_st) {
      continue;
    }
    // ignore the rear traffic in st graph
    const auto is_back_st =
        IsBackTrafficInSTProposal(obstacle->Id(), proposal.GetSTProposal());
    if (lc_conf.filter_back_obs_in_st && is_back_st) {
      continue;
    }

    const auto& o_box = obstacle->PerceptionBoundingBox();
    auto f_s{-1000.0}, b_s{1000.0};
    // get the min front/back s distance
    for (const auto& corner : o_box.GetAllCorners()) {
      const std::uint32_t nearest_pt_idx = trajectory.QueryNearestPoint(corner);
      const std::pair<double, double> obstacle_sl = ComputePositionProjection(
          corner.x(), corner.y(), trajectory.data()[nearest_pt_idx]);
      f_s = std::max(f_s, obstacle_sl.first);
      b_s = std::min(b_s, obstacle_sl.first);
    }
    auto min_ds = std::max(b_s - e_l_half, -e_l_half - f_s);
    min_ds = std::max(0.0, min_ds);

    // check if dangerous side obs
    // if the obs is overlap with ego at s dimension
    if (min_ds <= lc_conf.min_front_dist) {
      dangerous_reason = std::string("Too close Obs Dangerous LC") +
                         " distance: " + std::to_string(min_ds) +
                         " speed: " + std::to_string(obstacle->speed()) +
                         " id: " + obstacle->Id() + ",region " +
                         std::to_string(static_cast<int>(ort));
      dangerous_cost = lc_conf.max_cost;
      break;
    }
    // rear traffic
    if (f_s < 0.0 && min_ds <= lc_conf.max_back_dist) {
      // check whether the rear obstacle is too close
      if (min_ds <= lc_conf.min_back_dist) {
        dangerous_reason = std::string("Too closes rear Obs Dangerous LC") +
                           " distance: " + std::to_string(min_ds) +
                           " speed: " + std::to_string(obstacle->speed()) +
                           " id: " + obstacle->Id() + ",region " +
                           std::to_string(static_cast<int>(ort));
        dangerous_cost = lc_conf.max_cost;
        break;
      }

      //
      auto delta_v = obstacle->speed() - ego_pt.v();
      // if the obstacle is slower than the ego
      if (delta_v < lc_conf.delta_v_buf) {
        continue;
      }
      delta_v += lc_conf.delta_v_buf * 0.3;
      const auto delta_s = std::max(0.0, min_ds - lc_conf.min_back_dist);
      auto catch_up_time = std::min(delta_s / delta_v, lc_conf.catch_up_t_max);
      // const auto t_rate = catch_up_time / lc_conf.catch_up_t_max;
      const auto catch_up_coef = lc_time_table_.Evaluate(catch_up_time);
      const auto asb_v_coef =
          obstacle->speed() >= lc_conf.v_coef_velocity ? lc_conf.v_coef : 1.0;
      const auto obs_dangerous_cost = std::min(
          lc_conf.max_cost, catch_up_coef * asb_v_coef * lc_conf.max_cost);
      if (obs_dangerous_cost > dangerous_cost) {
        dangerous_reason =
            std::string("Rear Obs Dangerous LC") +
            " distance: " + std::to_string(min_ds) +
            " delta_v: " + std::to_string(delta_v - lc_conf.delta_v_buf * 0.3) +
            " id: " + obstacle->Id() + ",region " +
            std::to_string(static_cast<int>(ort)) +
            ", catch_up_time: " + std::to_string(catch_up_time);
        dangerous_cost = obs_dangerous_cost;
      }
    }  // rear traffic
  }    // obstacles loop

  if (dangerous_cost > kCostEpsilon) {
    costs.emplace_back(std::make_pair(dangerous_cost, dangerous_reason));
  }
}

void SafetyEvaluator::EvaluateCollisionCostObstacle(
    const CorridorInfo::Type type, const DiscretizedTrajectory& trajectory,
    const ::common::TrajectoryPoint& trajectory_point,
    const ::math::Box2d& adc_box, const Obstacle& obstacle,
    const common::VehicleParam& ego_param, double& max_collision_cost,
    std::pair<double, double>& max_cost_sl, double& max_cost_time,
    std::string& max_cost_obs_id) {
  if (obstacle.IsVirtual()) {
    return;
  }

  const bool is_lk = type == CorridorInfo::Type::LANE_KEEP;
  const bool is_stage = type == CorridorInfo::Type::STAGING;
  const bool is_lc = type == CorridorInfo::Type::LANE_CHANGE;
  const bool is_side_obs =
      (obstacle.RelativeRegionToLocalRoute() == RelativeRegionType::LEFT ||
       obstacle.RelativeRegionToLocalRoute() == RelativeRegionType::RIGHT);
  const bool is_front_side_obs = (obstacle.RelativeRegionToLocalRoute() ==
                                      RelativeRegionType::LEFT_FRONT ||
                                  obstacle.RelativeRegionToLocalRoute() ==
                                      RelativeRegionType::RIGHT_FRONT);

  const double ego_s = trajectory_point.path_point().s();
  const double ego_l = 0.0;
  const double time = trajectory_point.relative_time();
  const auto& obstacle_point = obstacle.GetPointAtTime(time);
  const auto& obstacle_bbox = obstacle.GetBoundingBox(obstacle_point);
  // just go through all four corners to calculate the sl
  const auto& sl = GetObstacleBoxSL(obstacle_bbox, trajectory);
  const auto [ds, dl] = GetSLDistanceOfObstacle(
      sl, ego_s, ego_l, ego_param.length(), ego_param.width());
  const double lon_cost = safety_lon_cost_table_.Evaluate(std::abs(ds));
  const double lat_cost = safety_lat_cost_table_.Evaluate(std::abs(dl));
  double collision_cost = std::min(lon_cost, lat_cost) *
                          collision_time_weight_table_.Evaluate(time);

  // check if the traffic is at side and the corridor is lk/staging
  if (is_lk || is_stage) {
    // header_ds - the delta distance between ego header and traffic header
    // along s
    const double header_ds = sl.end_s() - (ego_s + ego_param.length() * 0.5);
    if (header_ds <= 0.0) {
      return;
    }
    // only effect side traffic
    if (is_side_obs) {
      collision_cost *= config_.lk_near_side_front_collision_coef;
    } else if (is_front_side_obs) {
      collision_cost *= config_.lk_far_side_front_collision_coef;
    }
  }

  // check if lc corridor and the collision cost larger than a value(7.5 for
  // example) then just max the collision cost
  if (is_lc && collision_cost >= config_.lc_max_collision_dangerous_cost) {
    collision_cost = 10.0;
  }

  if (max_collision_cost < collision_cost) {
    max_collision_cost = collision_cost;
    max_cost_sl = std::make_pair(ds, dl);
    max_cost_time = time;
    max_cost_obs_id = obstacle.Id();
  }
}

void SafetyEvaluator::SlackSliceCost(const Eigen::MatrixXd& slack_slice,
                                     const std::vector<double>& weight,
                                     const LookupTable& t_weight_tbl,
                                     const std::vector<std::string>& name,
                                     Costs& costs) {
  // col size of slack_slice is 41, and total time is 4s
  for (int row = 0; row < slack_slice.rows(); ++row) {
    double cost{0.0};
    constexpr double interval_time = 0.2;
    for (int col = 0; col < slack_slice.cols(); ++col) {
      const double t_weight = t_weight_tbl.Evaluate(col * interval_time);
      const double slice_cost =
          slack_slice(row, col) * weight.at(row) * t_weight;
      if (slice_cost > kSlackEpsilon) {
        cost = std::max(cost, slice_cost);
      }
    }
    if (cost > kCostEpsilon) {
      const auto reason = "stiff constraint violation in " + name.at(row);
      costs.emplace_back(std::make_pair(cost, reason));
    }
  }
}

void SafetyEvaluator::SlackCost(
    const MPCData::Slacks::Slack& slack,
    const EvaluationDeciderConfig::SlackWeight& s_weight,
    const LookupTable& t_weight_tbl, const MPCData::Names& names,
    Costs& costs) {
  SlackSliceCost(slack.x, s_weight.weight_x, t_weight_tbl, names.x, costs);
  SlackSliceCost(slack.u, s_weight.weight_u, t_weight_tbl, names.u, costs);
  SlackSliceCost(slack.u_dot, s_weight.weight_u_dot, t_weight_tbl, names.u_dot,
                 costs);
}

SLBoundary SafetyEvaluator::GetObstacleBoxSL(
    const ::math::Box2d& obs, const DiscretizedTrajectory& trajectory) {
  auto f_s{-1000.0}, b_s{1000.0}, l_l{-1000.0}, r_l{1000.0};
  // get the min front/back s distance
  for (const auto& corner : obs.GetAllCorners()) {
    const std::uint32_t nearest_pt_idx = trajectory.QueryNearestPoint(corner);
    const std::pair<double, double> obstacle_sl = ComputePositionProjection(
        corner.x(), corner.y(), trajectory.data()[nearest_pt_idx]);
    f_s = std::max(f_s, obstacle_sl.first);
    b_s = std::min(b_s, obstacle_sl.first);
    l_l = std::max(l_l, obstacle_sl.second);
    r_l = std::min(r_l, obstacle_sl.second);
  }

  auto sl = SLBoundary{};
  sl.set_start_s(b_s);
  sl.set_end_s(f_s);
  sl.set_start_l(r_l);
  sl.set_end_l(l_l);

  return sl;
}

// ds < 0 - front obs, ds > 0 - back obs, ds = 0 - overlap along s
// dl < 0 - left obs, ls > 0 - right obs, ls = 0 - overlap along l
std::pair<double, double> SafetyEvaluator::GetSLDistanceOfObstacle(
    const SLBoundary& sl_obs, const double s, const double l,
    const double length, const double width) {
  double d_s = 0.0, d_l = 0.0;
  const auto h_l = length * 0.5, h_w = width * 0.5;
  // ego back to obs front, ego front to obs back
  const auto eb_to_of = (s - h_l) - sl_obs.end_s();
  const auto ef_to_ob = (s + h_l) - sl_obs.start_s();
  // obstacle is somehow front / obstacle is somehow back
  if (s <= sl_obs.start_s()) {
    d_s = std::min(0.0, ef_to_ob);
  } else if (s >= sl_obs.end_s()) {
    d_s = std::max(0.0, eb_to_of);
  }

  // ego right to obs left, ego left to obs right
  const auto er_to_ol = (l - h_w) - sl_obs.end_l();
  const auto el_to_or = (l + h_w) - sl_obs.start_l();
  // obstacle is somehow left / obstacle is somehow right
  if (l <= sl_obs.start_l()) {
    d_l = std::min(0.0, el_to_or);
  } else if (l >= sl_obs.end_l()) {
    d_l = std::max(0.0, er_to_ol);
  }

  return std::make_pair(d_s, d_l);
}

}  // namespace planning
}  // namespace zark
