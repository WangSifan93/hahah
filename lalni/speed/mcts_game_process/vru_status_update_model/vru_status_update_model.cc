#include "vru_status_update_model.h"

#include <limits>
#include <map>

#include "math/double.h"

namespace e2e_noa::planning {

namespace {
constexpr double kEpsilon = 1e-6;
constexpr double k2PI = 2.0 * M_PI;
constexpr bool kUSEHRVO = false;
constexpr bool kUSETURN = true;
struct Action {
  double act_jerk = 0.0;
  double act_angular_v = 0.0;
};

void SampleActions(const AgentStatus &vru_parent_status,
                   const SpeedPlanningParamsProto::MCTSVruConfigProto &params,
                   const StOverlapMetaProto::OverlapSource &overlap_source,
                   const StBoundaryProto::ObjectType &vru_type,
                   const double heading_diff, const double original_v,
                   const InteractionZone interaction_zone,
                   std::vector<Action> &actions) {
  actions.clear();
  const int total_count = (2 * params.num_jerk_samples() + 1) *
                          (2 * params.num_ang_v_samples() + 1);
  actions.reserve(total_count);

  const double v = vru_parent_status.v;

  // 计算jerk缩放因子
  double jerk_factor = 1.0;
  constexpr double v_max_pedestrian = 3.0;
  constexpr double v_high_pedestrian = 2.0;
  constexpr double v_lat_lower = 1.5;
  const double v_low = params.v_low();
  double v_high = params.v_high();
  double v_max = params.v_max();
  if (vru_type == StBoundaryProto::PEDESTRIAN) {
    v_high = v_high_pedestrian;
    v_max = v_max_pedestrian;
  }
  if (v <= v_low) {
    jerk_factor =
        std::min(v / (0.5 * std::max(params.jerk_max(), kEpsilon)), 1.0);
  } else if (v >= v_high) {
    jerk_factor = params.epsilon_j();
  } else {
    jerk_factor = (v_high - v) / std::max(v_high - v_low, kEpsilon);
    jerk_factor = std::min(std::max(0.5, jerk_factor), 1.0);
  }

  // 计算角加速度缩放因子
  const double ang_v_factor =
      std::min(1.0, std::max(0.0, (v_max - v) / std::max(v_max, kEpsilon)));

  // 计算动态采样范围
  const double dynamic_jerk_max = jerk_factor * params.jerk_max();
  const double dynamic_ang_v_max = ang_v_factor * params.ang_v_max();

  // 计算采样步长
  const double jerk_step =
      (dynamic_jerk_max) / std::max(1, (params.num_jerk_samples() - 1));
  const double ang_v_step =
      (dynamic_ang_v_max) / std::max(1, (params.num_ang_v_samples() - 1));
  const bool is_turn_left = (interaction_zone == InteractionZone::TurnLeft);
  const bool only_lon =
      ((original_v <= v_lat_lower) && (heading_diff < 2 * M_PI / 3.0) &&
       (heading_diff > M_PI / 3.0)) ||
      (overlap_source != StOverlapMetaProto::OBJECT_CUTIN && !is_turn_left);
  // 生成候选动作
  for (int i = -params.num_jerk_samples(); i <= params.num_jerk_samples();
       ++i) {
    const double jerk = i * jerk_step;
    if (ad_e2e::planning::math::Double::Compare(original_v, params.v_high()) ==
            ad_e2e::planning::math::Double::CompareType::GREATER ||
        only_lon) {
      Action action;
      action.act_jerk = jerk;
      action.act_angular_v = 0.0;
      actions.push_back(action);
      continue;
    }
    for (int j = -params.num_ang_v_samples(); j <= params.num_ang_v_samples();
         ++j) {
      const double ang_v = j * ang_v_step;
      Action action;
      action.act_jerk = jerk;
      action.act_angular_v = ang_v;

      actions.push_back(action);
    }
  }
}

bool PredictState(const AgentStatus &vru_parent_status, const Action &action,
                  const double time_step, AgentStatus &vru_child_status) {
  vru_child_status = vru_parent_status;
  const double t_step = 0.1;

  if (t_step > time_step) {
    return false;
  }
  for (double t = t_step; t <= time_step; t += t_step) {
    const double cos_theta = std::cos(vru_child_status.heading);
    const double sin_theta = std::sin(vru_child_status.heading);

    const double dx =
        vru_child_status.v * cos_theta * t_step +
        0.5 * vru_child_status.a * cos_theta * t_step * t_step +
        (1.0 / 6.0) * action.act_jerk * cos_theta * std::pow(t_step, 3);
    vru_child_status.x += dx;

    const double dy =
        vru_child_status.v * sin_theta * t_step +
        0.5 * vru_child_status.a * sin_theta * t_step * t_step +
        (1.0 / 6.0) * action.act_jerk * sin_theta * std::pow(t_step, 3);
    vru_child_status.y += dy;

    vru_child_status.v +=
        vru_child_status.a * t_step + 0.5 * action.act_jerk * t_step * t_step;
    vru_child_status.v = std::max(0.0, vru_child_status.v);

    vru_child_status.a += action.act_jerk * t_step;

    vru_child_status.heading += vru_child_status.angular_v * t_step;
    vru_child_status.heading = std::fmod(vru_child_status.heading, k2PI);
    if (vru_child_status.heading > M_PI) {
      vru_child_status.heading -= k2PI;
    } else if (vru_child_status.heading < -M_PI) {
      vru_child_status.heading += k2PI;
    }

    vru_child_status.angular_v = action.act_angular_v;
  }
  return true;
}

bool CaculateDistance2Box(double &min_distance, const Box2d &leader_box,
                          const Vec2d &follower_point,
                          const double follower_heading) {
  if (leader_box.IsPointIn(follower_point)) {
    return false;
  }
  bool is_intersect = false;
  auto RayRectangleIntersection = [&is_intersect](const Segment2d &edge,
                                                  const Vec2d &point,
                                                  const double heading) {
    const double dir_x = cos(heading);
    const double dir_y = sin(heading);

    const double x1 = point.x();
    const double y1 = point.y();
    const double x2 = edge.start().x();
    const double y2 = edge.start().y();
    const double x3 = edge.end().x();
    const double y3 = edge.end().y();

    const double denominator = (y3 - y2) * dir_x - (x3 - x2) * dir_y;
    if (ad_e2e::planning::math::Double::Compare(denominator, 0.0) ==
        ad_e2e::planning::math::Double::CompareType::EQUAL) {
      // 射线与线段平行
      is_intersect = false;
      return 0.0;
    }

    const double t =
        ((y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2)) / denominator;
    const double u = ((y2 - y1) * dir_x - (x2 - x1) * dir_y) / denominator;

    if (t >= 0 && u >= 0 && u <= 1) {
      const double intersection_x = x1 + t * dir_x;
      const double intersection_y = y1 + t * dir_y;
      is_intersect = true;
      return hypot(intersection_x - point.x(), intersection_y - point.y());
    } else {
      is_intersect = false;
      return 0.0;
    }
  };
  const auto edges = leader_box.GetEdgesCounterClockwise();
  bool has_intersect = false;
  for (const auto &edge : edges) {
    const double distance =
        RayRectangleIntersection(edge, follower_point, follower_heading);
    if (is_intersect) {
      min_distance = std::min(min_distance, distance);
      has_intersect = true;
    }
  }
  return has_intersect;
}

bool FindIntersectionAndCalculateForStraight(
    const AgentStatus &vru_child_status, const AgentStatus &leader_status,
    const double ego_width, const double follower_width,
    IntersectionResult &intersection_result) {
  const double follower_x = vru_child_status.x;
  const double follower_y = vru_child_status.y;
  const double follower_heading = vru_child_status.heading;
  const double follower_speed = vru_child_status.v;
  const double leader_x = leader_status.x;
  const double leader_y = leader_status.y;
  const double leader_heading = leader_status.heading;
  const double leader_speed = leader_status.v;
  const double caculate_t = 10.0;
  const double ray_len =
      std::max(follower_speed * caculate_t, leader_speed * caculate_t);
  if (ray_len < 1.0) {
    return false;
  }
  if (follower_speed < kEpsilon) {
    return false;
  }
  auto TranslatePoint = [](const double dx, const double dy, const double theta,
                           const Vec2d &pos, const Vec2d &base_point,
                           const double ray_len) {
    Vec2d pos1 = pos + Vec2d(dx * cos(theta) - dy * sin(theta),
                             dy * cos(theta) + dx * sin(theta));
    Vec2d pos2 = pos + Vec2d(dx * cos(theta) + dy * sin(theta),
                             -dy * cos(theta) + dx * sin(theta));
    Segment2d seg_1 = Segment2d(ray_len, pos1, Vec2d::FastUnitFromAngle(theta));
    Segment2d seg_2 = Segment2d(ray_len, pos2, Vec2d::FastUnitFromAngle(theta));
    if (seg_1.DistanceTo(base_point) < seg_2.DistanceTo(base_point)) {
      return pos1;
    } else {
      return pos2;
    }
  };
  Vec2d follower_pos(follower_x, follower_y);
  Vec2d leader_pos(leader_x, leader_y);
  Vec2d follower_point =
      TranslatePoint(0.0, follower_width * 0.5, follower_heading, follower_pos,
                     leader_pos, ray_len);
  constexpr double ego_buffer = 0.2;
  Vec2d leader_point =
      TranslatePoint(-6.0, ego_width * 0.5 + ego_buffer, leader_heading,
                     leader_pos, follower_pos, ray_len);
  Vec2d follower_unit_direct = Vec2d::FastUnitFromAngle(follower_heading);
  Vec2d leader_unit_direct = Vec2d::FastUnitFromAngle(leader_heading);
  Segment2d follower_segment(ray_len, follower_point, follower_unit_direct);
  Segment2d leader_segment(ray_len, leader_point, leader_unit_direct);
  Vec2d intersect_point;
  constexpr double min_follower_speed = 0.3;
  if (leader_segment.GetIntersect(follower_segment, &intersect_point)) {
    intersection_result.has_intersection = true;
    intersection_result.dis_leader = intersect_point.DistanceTo(leader_point);
    intersection_result.dis_follower =
        intersect_point.DistanceTo(follower_point);
    intersection_result.t_leader_arrive =
        (leader_speed > 0) ? intersection_result.dis_leader / leader_speed : 0;
    intersection_result.t_follower_arrive =
        (follower_speed > min_follower_speed)
            ? intersection_result.dis_follower / follower_speed
            : intersection_result.dis_follower / min_follower_speed;
    return true;
  }
  return false;
}

bool FindIntersectionAndCalculateForJunctionTurn(
    const AgentStatus &vru_child_status, const AgentStatus &leader_status,
    const double ego_width, const double follower_width,
    const DiscretizedPath &ego_path, IntersectionResult &intersection_result) {
  if (ego_path.size() < 2) {
    return false;
  }
  const double caculate_t = 8.0;
  const double leader_speed = leader_status.v;
  const double follower_speed = vru_child_status.v;
  const double ray_len = std::max(vru_child_status.v * caculate_t, 0.0);
  if (ray_len < 1.0) {
    return false;
  }
  if (follower_speed < kEpsilon) {
    return false;
  }
  const double cos_theta = cos(vru_child_status.heading);
  const double sin_theta = sin(vru_child_status.heading);
  constexpr double point_num = 4.0;
  const double step = ray_len / point_num;
  Vec2d pre_point = Vec2d(vru_child_status.x, vru_child_status.y);
  Vec2d follower_start_pos(vru_child_status.x, vru_child_status.y);
  FrenetCoordinate pre_sl = ego_path.XYToSL(pre_point);
  bool has_intersect = false;
  Vec2d intersect_point;
  for (double t = 0.0; t <= ray_len; t += step) {
    Vec2d follower_pos;
    follower_pos.set_x(vru_child_status.x + t * cos_theta);
    follower_pos.set_y(vru_child_status.y + t * sin_theta);
    const FrenetCoordinate follower_sl = ego_path.XYToSL(follower_pos);
    const double l_limit = (ego_width / 2.0 + follower_width / 2.0);
    if (fabs(follower_sl.l) < l_limit || follower_sl.l * pre_sl.l < kEpsilon) {
      has_intersect = true;
      const double dl = std::max(fabs(pre_sl.l - follower_sl.l), kEpsilon);
      const double distance = step * (l_limit - fabs(follower_sl.l)) / dl;
      intersect_point.set_x(follower_pos.x() - distance * cos_theta);
      intersect_point.set_y(follower_pos.y() - distance * sin_theta);
      break;
    }
    pre_point = follower_pos;
  }

  Vec2d leader_pos(leader_status.x, leader_status.y);
  constexpr double min_follower_speed = 0.4;
  if (has_intersect) {
    intersection_result.has_intersection = true;
    intersection_result.dis_follower =
        intersect_point.DistanceTo(follower_start_pos);
    intersection_result.t_follower_arrive =
        (follower_speed > min_follower_speed)
            ? intersection_result.dis_follower / follower_speed
            : intersection_result.dis_follower / min_follower_speed;
    return true;
  }
  return false;
}

double CheckCollision(
    const AgentStatus &vru_child_status, const AgentStatus &leader_status,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params,
    const double follower_length, const double follower_width,
    const InteractionZone interaction_zone, const DiscretizedPath &ego_path,
    const std::vector<const SpacetimeObjectTrajectory *> &other_agents) {
  if (kUSEHRVO) {
    // 计算相对位置
    const double p_rel_x = vru_child_status.x - leader_status.x;
    const double p_rel_y = vru_child_status.y - leader_status.y;

    // 计算速度分量
    const double ego_vx = leader_status.v * std::cos(leader_status.heading);
    const double ego_vy = leader_status.v * std::sin(leader_status.heading);

    const double vru_vx =
        vru_child_status.v * std::cos(vru_child_status.heading);
    const double vru_vy =
        vru_child_status.v * std::sin(vru_child_status.heading);

    // 计算混合责任速度
    const double v_mix_x =
        (1 - params.beta()) * ego_vx + params.beta() * vru_vx;
    const double v_mix_y =
        (1 - params.beta()) * ego_vy + params.beta() * vru_vy;

    // 计算相对速度分量
    const double v_ref_x = vru_vx - v_mix_x;
    const double v_ref_y = vru_vy - v_mix_y;

    Box2d ego_box =
        Box2d(Vec2d(leader_status.x, leader_status.y), leader_status.heading,
              params.ego_length(), params.ego_width());
    Box2d vru_box =
        Box2d(Vec2d(vru_child_status.x, vru_child_status.y),
              vru_child_status.heading, follower_length, follower_width);

    const double v_ref_heading = std::atan2(v_ref_y, v_ref_x);
    double min_distance = std::numeric_limits<double>::infinity();
    double t = std::numeric_limits<double>::infinity();
    for (const auto &corners : vru_box.GetAllCorners()) {
      double temp_distance = std::numeric_limits<double>::infinity();
      if (CaculateDistance2Box(temp_distance, ego_box, corners,
                               v_ref_heading)) {
        if (temp_distance < min_distance) {
          min_distance = temp_distance;
          t = min_distance / (hypot(v_ref_x, v_ref_y) + kEpsilon);
        }
      }
    }
    return t;
  } else {
    const bool is_stright =
        (interaction_zone == InteractionZone::Straight ||
         interaction_zone == InteractionZone::JunctionStraight);
    IntersectionResult intersection_result;
    if (is_stright || !kUSETURN) {
      if (FindIntersectionAndCalculateForStraight(
              vru_child_status, leader_status, follower_length,
              params.ego_width(), intersection_result)) {
        return intersection_result.t_follower_arrive;
      } else {
        return std::numeric_limits<double>::infinity();
      }
    } else {
      if (FindIntersectionAndCalculateForJunctionTurn(
              vru_child_status, leader_status, follower_length,
              params.ego_width(), ego_path, intersection_result)) {
        return intersection_result.t_follower_arrive;
      } else {
        return std::numeric_limits<double>::infinity();
      }
    }
  }
  return std::numeric_limits<double>::infinity();
}

double CalculateSafetyCost(
    const double tau_min,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params) {
  constexpr double tau_thresh = 8.0;
  if (tau_min > tau_thresh) {
    return 0.0;
  }
  double safety_cost = params.w_safety() * std::exp(-params.k() * tau_min);
  return std::max(safety_cost, 0.0);
}

double CalculateGoalCost(
    const AgentStatus &vru_child_status, const double goal_heading,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params) {
  double goal_cost =
      params.w_goal() * tan(vru_child_status.heading - goal_heading);
  return std::clamp(goal_cost, 0.0, 100.0);
}

double CalculateSpeedCost(
    const AgentStatus &vru_child_status, const double goal_speed,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params) {
  const double diff = vru_child_status.v - goal_speed;
  double speed_cost = params.w_speed() * (diff * diff);
  return std::clamp(speed_cost, 0.0, 3.0);
}

double CalculateAccelCost(
    const AgentStatus &vru_child_status,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params) {
  double acc_cost =
      params.w_accel() * (vru_child_status.a * vru_child_status.a);
  return std::clamp(acc_cost, 0.0, 3.0);
}

double CalculateJerkCost(
    const Action &action,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params) {
  double jerk_cost = params.w_jerk() * (action.act_jerk * action.act_jerk);
  return std::clamp(jerk_cost, 0.0, 3.0);
}

double CalculateAngSpeedCost(
    const AgentStatus &vru_child_status,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params) {
  double angular_v_cost = params.w_angular_v() * (vru_child_status.angular_v *
                                                  vru_child_status.angular_v);
  return std::clamp(angular_v_cost, 0.0, 10.0);
}

double CalculateAngAccelCost(
    const Action &action,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params) {
  double ang_accel_cost =
      params.w_angular_a() * (action.act_angular_v * action.act_angular_v);
  return std::clamp(ang_accel_cost, 0.0, 3.0);
}

double CalculateTotalCost(
    const AgentStatus &vru_child_status, const Vec2d &goal_position,
    const double goal_speed, const double goal_heading,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params,
    const double tau_min, const Action &action,
    std::shared_ptr<SptDebug> spt_debug) {
  const double safety_cost = CalculateSafetyCost(tau_min, params);
  const double goal_cost =
      CalculateGoalCost(vru_child_status, goal_heading, params);
  const double speed_cost =
      CalculateSpeedCost(vru_child_status, goal_speed, params);
  const double acc_cost = CalculateAccelCost(vru_child_status, params);
  const double jerk_cost = CalculateJerkCost(action, params);
  const double angular_v_cost = CalculateAngSpeedCost(vru_child_status, params);

  const double ang_accel_cost = 0.0;
  const double total_reward = safety_cost + goal_cost + speed_cost + acc_cost +
                              jerk_cost + angular_v_cost + ang_accel_cost;

  if (spt_debug != nullptr) {
    auto &action_and_cost = spt_debug->vru_action_and_costs.emplace_back();
    action_and_cost.jerk = action.act_jerk;
    action_and_cost.angular_a = action.act_angular_v;
    action_and_cost.SafetyCost = safety_cost;
    action_and_cost.GoalCost = goal_cost;
    action_and_cost.SpeedCost = speed_cost;
    action_and_cost.AccelCost = acc_cost;
    action_and_cost.JerkCost = jerk_cost;
    action_and_cost.AngSpeedCost = angular_v_cost;
    action_and_cost.AngAccelCost = ang_accel_cost;
    action_and_cost.TotalCost = total_reward;
    action_and_cost.tau_min = tau_min;
  }

  return total_reward;
}
}  // namespace

bool UpdateVRUStatus(
    const AgentStatus &leader_status, const AgentStatus &vru_parent_status,
    const Vec2d &goal_position, const double goal_speed,
    const double goal_heading, const double original_v,
    const double current_time, const double time_step,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params,
    const StOverlapMetaProto::OverlapSource &overlap_source,
    const StBoundaryProto::ObjectType &vru_type,
    const std::vector<const SpacetimeObjectTrajectory *> &other_agents,
    const double follower_length, const double follower_width,
    const double heading_diff, AgentStatus &vru_child_status,
    const InteractionZone interaction_zone, const DiscretizedPath &ego_path,
    std::shared_ptr<SptDebug> spt_debug) {
  std::vector<Action> candidate_actions;
  SampleActions(vru_parent_status, params, overlap_source, vru_type,
                heading_diff, original_v, interaction_zone, candidate_actions);
  if (candidate_actions.empty()) {
    DLOG(ERROR) << "no candidate actions";
    return false;
  }
  double min_cost = std::numeric_limits<double>::max();
  bool is_valid = false;
  for (const auto &action : candidate_actions) {
    AgentStatus vru_child_tmp_status;
    if (!PredictState(vru_parent_status, action, time_step,
                      vru_child_tmp_status)) {
      continue;
    }
    const double tau_min = CheckCollision(
        vru_child_tmp_status, leader_status, params, follower_length,
        follower_width, interaction_zone, ego_path, other_agents);
    const double cost =
        CalculateTotalCost(vru_child_tmp_status, goal_position, goal_speed,
                           goal_heading, params, tau_min, action, spt_debug);
    if (cost < min_cost) {
      min_cost = cost;
      vru_child_status = std::move(vru_child_tmp_status);
      vru_child_status.act_jerk = action.act_jerk;
      vru_child_status.angular_v = action.act_angular_v;
      is_valid = true;
    }
  }
  return is_valid;
}

}  // namespace e2e_noa::planning
