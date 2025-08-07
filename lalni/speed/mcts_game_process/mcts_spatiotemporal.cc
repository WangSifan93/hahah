#include "mcts_spatiotemporal.h"

namespace e2e_noa::planning {
constexpr double kRadio = 0.6;
constexpr double kEpsilon = 1e-3;

void MCTSSpatiotemporal::InitMCTSSpatiotemporal(
    const double min_v, const double max_v, const double time_step,
    const double jerk, const double leader_v, const double follower_v,
    const DiscretizedPath* leader_path, const double goal_v,
    const double goal_pos_x, const double goal_pos_y, const double goal_heading,
    const double leader_x, const double leader_y, const double follower_x,
    const double follower_y, const double follower_length,
    const double follower_width, const double heading_diff,
    const StOverlapMetaProto::OverlapSource& overlap_source,
    const StBoundaryProto::ObjectType& vru_type,
    const std::vector<const SpacetimeObjectTrajectory*>& other_agents) {
  leader_cur_v_ = leader_v;
  follower_cur_v_ = follower_v;
  leader_max_v_ = std::max(max_v, kEpsilon);
  time_step_ = time_step;
  leader_model_ = VehicleSimulationModel(min_v, max_v, time_step, jerk);
  leader_path_ = leader_path;
  goal_v_ = goal_v;
  goal_position_ = Vec2d(goal_pos_x, goal_pos_y);
  goal_heading_ = goal_heading;
  leader_start_x_ = leader_x;
  leader_start_y_ = leader_y;
  follower_start_x_ = follower_x;
  follower_start_y_ = follower_y;
  follower_length_ = follower_length;
  follower_width_ = follower_width;
  heading_diff_ = heading_diff;
  other_agents_ = other_agents;
  overlap_source_ = overlap_source;
  vru_type_ = vru_type;
}

bool MCTSSpatiotemporal::PrePruning(const MCTSNodePtr& node,
                                    const double leader_action) {
  if (nullptr == node || nullptr == params_) {
    DLOG(INFO) << "PrePruning nullptr == node || nullptr == params_ "
               << bool(nullptr == params_) << " " << bool(nullptr == node);
    return true;
  }
  const double cur_a = node->state.leader_status.a;
  const double cur_v = node->state.leader_status.v;
  const double new_a = cur_a + leader_action;
  if (new_a > params_->vru_reward_params().a_max() ||
      new_a < params_->vru_reward_params().a_min()) {
    return true;
  }
  if (ad_e2e::planning::math::Double::Compare(cur_v, leader_max_v_) ==
          ad_e2e::planning::math::Double::CompareType::GREATER &&
      new_a > 0.0) {
    return true;
  }
  if (cur_v < leader_max_v_ &&
      new_a > params_->vru_reward_params().a_max() *
                  (1.0 - cur_v / leader_max_v_ * kRadio)) {
    return true;
  }
  return false;
}

void MCTSSpatiotemporal::CreateChildNode(const MCTSNodePtr& parent,
                                         const double leader_action,
                                         MCTSNodePtr& child) {
  if (nullptr == parent) {
    DLOG(INFO) << "CreateChildNode nullptr == parent";
    return;
  }
  NodeState state;
  UpdateLeaderState(parent, leader_action, state);

  child = std::make_shared<MCTSNode>(state, parent);
  child->current_time = parent->current_time + time_step_;
}

void MCTSSpatiotemporal::UpdateChildNode(const MCTSNodeConstPtr& parent,
                                         const MCTSNodePtr& child) {
  if (nullptr == parent || nullptr == child) {
    return;
  }
  NodeState& state = child->state;
  const double current_time = child->current_time;
  std::shared_ptr<SptDebug> spt_debug = std::make_shared<SptDebug>();
  if (!UpdateVRUStatus(
          state.leader_status, parent->state.follower_status, goal_position_,
          goal_v_, goal_heading_, follower_cur_v_, current_time, time_step_,
          params_->vru_config(), overlap_source_, vru_type_, other_agents_,
          follower_length_, follower_width_, heading_diff_,
          state.follower_status, interaction_zone_, ego_path_, spt_debug)) {
    return;
  }
  child->is_leaf = IsLeaf(child);
  child->spt_debug = spt_debug;
}

bool MCTSSpatiotemporal::IsLeaf(const MCTSNodePtr& node) {
  if (nullptr == node || nullptr == params_ || nullptr == leader_path_) {
    return false;
  }
  Vec2d follower_pos(node->state.follower_status.x,
                     node->state.follower_status.y);
  const auto follower_sl = leader_path_->XYToSL(follower_pos);
  const auto follower_start_sl =
      leader_path_->XYToSL(Vec2d(follower_start_x_, follower_start_y_));
  node->state.follower_status.s = follower_sl.s;
  const double safe_buffer = 5.0;
  if (follower_sl.s + safe_buffer < node->state.leader_status.s) {
    return true;
  }
  const bool is_vru_cross_path =
      ((follower_start_sl.l < -params_->vru_reward_params().leader_width() &&
        follower_sl.l > params_->vru_reward_params().leader_width()) ||
       (follower_start_sl.l > params_->vru_reward_params().leader_width() &&
        follower_sl.l < -params_->vru_reward_params().leader_width()));
  if (is_vru_cross_path) {
    return true;
  }
  if (std::abs(follower_sl.l) < params_->vru_reward_params().leader_width() &&
      follower_sl.s < node->state.leader_status.s +
                          params_->vru_reward_params().collision_distance()) {
    return true;
  }

  if (ad_e2e::planning::math::Double::Compare(node->state.leader_status.v,
                                              0.0) ==
          ad_e2e::planning::math::Double::CompareType::EQUAL ||
      (ad_e2e::planning::math::Double::Compare(node->state.follower_status.v,
                                               0.0) ==
       ad_e2e::planning::math::Double::CompareType::EQUAL)) {
    return true;
  }

  const double max_time = time_step_ * 9.0;
  if (node->current_time > max_time) {
    return true;
  }
  return false;
}

void MCTSSpatiotemporal::UpdateLeaderState(const MCTSNodeConstPtr& parent,
                                           const double leader_action,
                                           NodeState& state) {
  if (nullptr == parent || nullptr == leader_path_) {
    DLOG(INFO) << "UpdateLeaderState nullptr == parent";
    return;
  }

  const double leader_a = parent->state.leader_status.a + leader_action;
  leader_model_.SetInitialState(parent->state.leader_status.s,
                                parent->state.leader_status.v, leader_a);
  state.leader_status.id = parent->state.leader_status.id;
  state.leader_status.s = leader_model_.ComputeDistance(time_step_);
  state.leader_status.v = leader_model_.ComputeVelocity(time_step_);
  state.leader_status.a = leader_a;
  state.leader_status.act_jerk = leader_action;
  state.leader_status.remain_dis =
      parent->state.leader_status.remain_dis -
      (state.leader_status.s - parent->state.leader_status.s);
  state.leader_arrive_time = leader_model_.ComputeArriveTime(
      state.leader_status.remain_dis + state.leader_status.s);
  const auto leader_point = leader_path_->Evaluate(state.leader_status.s);
  state.leader_status.x = leader_point.x();
  state.leader_status.y = leader_point.y();
  state.leader_status.heading = leader_point.theta();
}

void MCTSSpatiotemporal::UpdateNodeStepReward(const MCTSNodePtr& node) {
  if (nullptr == node || nullptr == params_) {
    node->step_reward = 0.0;
    return;
  }

  const double reward_self =
      params_->vru_reward_params().w_self() * CalculateEgoReward(node);
  const double reward_other =
      params_->vru_reward_params().w_other() * CalculateOtherReward(node);
  node->step_reward = reward_self + reward_other;
  if (enable_spt_debug_) {
    if (nullptr == node->spt_debug) {
      node->spt_debug = std::make_shared<SptDebug>();
    }
    node->spt_debug->total_reward = node->step_reward;
    node->spt_debug->reward_self = reward_self;
    node->spt_debug->reward_other = reward_other;
  }
}

double MCTSSpatiotemporal::CalculateEgoReward(const MCTSNodePtr& node) {
  if (nullptr == node || nullptr == params_) {
    return 0.0;
  }
  IntersectionResult intersection_result;
  FindIntersectionAndCalculate(
      node->state.follower_status.x, node->state.follower_status.y,
      node->state.follower_status.heading, node->state.follower_status.v,
      node->state.leader_status.x, node->state.leader_status.y,
      node->state.leader_status.v, intersection_result);

  const double reward_safe = params_->vru_reward_params().w_safe() *
                             CalculateSafetyReward(node, intersection_result);
  const double reward_eff =
      params_->vru_reward_params().w_eff() *
      CalculateEfficiencyReward(node, intersection_result);
  const double reward_acc =
      params_->vru_reward_params().w_acc() * CalculateAccelerationReward(node);
  const double reward_jerk =
      params_->vru_reward_params().w_jerk() * CalculateJerkReward(node);
  const double reward_act =
      params_->vru_reward_params().w_act() * CalculateActionSmoothReward(node);

  const double reward_self_step =
      params_->vru_reward_params().w_self_step() *
      (reward_safe + reward_eff + reward_acc + reward_jerk + reward_act);

  const double reward_self_end =
      params_->vru_reward_params().w_self_end() * CalculateEndReward(node);

  const double reward_self = reward_self_step + reward_self_end;
  if (enable_spt_debug_) {
    if (nullptr == node->spt_debug) {
      node->spt_debug = std::make_shared<SptDebug>();
    }
    node->spt_debug->reward_safe = reward_safe;
    node->spt_debug->reward_eff = reward_eff;
    node->spt_debug->reward_acc = reward_acc;
    node->spt_debug->reward_jerk = reward_jerk;
    node->spt_debug->reward_act = reward_act;
    node->spt_debug->reward_self_step = reward_self_step;
    node->spt_debug->reward_self_end = reward_self_end;
  }
  return reward_self;
}

double MCTSSpatiotemporal::CalculateOtherReward(const MCTSNodePtr& node) {
  if (nullptr == node || nullptr == params_) {
    return 0.0;
  }
  const double follower_v_current = node->state.follower_status.v;
  const double follower_heading_current = node->state.follower_status.heading;
  double follower_v_parent = follower_v_current;
  double follower_heading_parent = follower_heading_current;
  if (nullptr != node->GetParent()) {
    follower_v_parent = node->GetParent()->state.follower_status.v;
    follower_heading_parent = node->GetParent()->state.follower_status.heading;
  }
  const double follower_delta_v =
      (follower_v_current - follower_v_parent) / time_step_;
  const double follower_delta_heading =
      std::abs(follower_heading_current - follower_heading_parent) / time_step_;

  double r_follower_delta_v = 0.0;
  if (follower_delta_v < 0.0) {
    r_follower_delta_v =
        -std::exp(-1 * params_->vru_reward_params().beta_delta_v() *
                  follower_delta_v) +
        1.0;
  }
  const double reward_follower_delta_v =
      std::min(params_->vru_reward_params().reward_max(),
               std::max(params_->vru_reward_params().reward_min(),
                        params_->vru_reward_params().w_other_delta_v() *
                            r_follower_delta_v));
  const double reward_follower_delta_heading =
      std::min(params_->vru_reward_params().reward_max(),
               std::max(params_->vru_reward_params().reward_min(),
                        -params_->vru_reward_params().w_other_delta_heading() *
                            follower_delta_heading));
  const double reward_follower_v = std::min(
      params_->vru_reward_params().reward_max(),
      std::max(params_->vru_reward_params().reward_min(),
               params_->vru_reward_params().w_other_v() *
                   std::min(follower_v_current /
                                params_->vru_reward_params().v_max_vru(),
                            1.0)));
  const double reward_other = reward_follower_delta_v +
                              reward_follower_delta_heading + reward_follower_v;
  if (enable_spt_debug_) {
    if (nullptr == node->spt_debug) {
      node->spt_debug = std::make_shared<SptDebug>();
    }
    node->spt_debug->reward_follower_delta_v = reward_follower_delta_v;
    node->spt_debug->reward_follower_delta_heading =
        reward_follower_delta_heading;
    node->spt_debug->reward_follower_v = reward_follower_v;
  }
  return reward_other;
}

double MCTSSpatiotemporal::CalculateSafetyReward(
    const MCTSNodeConstPtr& node,
    const IntersectionResult& intersection_result) {
  if (nullptr == node || nullptr == params_) {
    return 0.0;
  }
  if (!intersection_result.has_intersection) {
    return params_->vru_reward_params().reward_max();
  }
  const double t_diff = intersection_result.t_leader_arrive -
                        intersection_result.t_follower_arrive;
  const double v_leader = node->state.leader_status.v;
  const double v_follower = node->state.follower_status.v;
  if (node->is_leaf) {
    if (intersection_result.dis_leader <
            params_->vru_reward_params().threshold_collision() &&
        intersection_result.dis_follower <
            params_->vru_reward_params().threshold_collision()) {
      return params_->vru_reward_params().reward_min();
    }
  }
  if (std::abs(t_diff) < params_->vru_reward_params().t_diff_max()) {
    return (std::abs(t_diff) / params_->vru_reward_params().t_diff_max()) *
           params_->vru_reward_params().reward_max();
  } else {
    return params_->vru_reward_params().reward_max();
  }
  return 0.0;
}

double MCTSSpatiotemporal::CalculateEfficiencyReward(
    const MCTSNodeConstPtr& node,
    const IntersectionResult& intersection_result) {
  if (nullptr == node || nullptr == params_) {
    return 0.0;
  }
  if (ad_e2e::planning::math::Double::Compare(
          params_->vru_reward_params().v_perf(), 0) ==
      ad_e2e::planning::math::Double::CompareType::EQUAL) {
    return 0.0;
  }
  if (!intersection_result.has_intersection) {
    double efficiency_reward = node->state.leader_status.v /
                               params_->vru_reward_params().v_perf() *
                               params_->vru_reward_params().reward_max();
    return std::min(
        params_->vru_reward_params().reward_max(),
        std::max(params_->vru_reward_params().reward_min(), efficiency_reward));
  }

  const double t_expected =
      intersection_result.dis_leader / params_->vru_reward_params().v_perf();
  if (intersection_result.t_leader_arrive < t_expected) {
    return params_->vru_reward_params().reward_max();
  }
  return (t_expected / intersection_result.t_leader_arrive) *
         params_->vru_reward_params().reward_max();
}

double MCTSSpatiotemporal::CalculateAccelerationReward(
    const MCTSNodeConstPtr& node) {
  if (nullptr == node || nullptr == params_) {
    return 0.0;
  }
  const double acc = node->state.leader_status.a;
  if (ad_e2e::planning::math::Double::Compare(acc, 0) !=
          ad_e2e::planning::math::Double::CompareType::LESS &&
      acc < params_->vru_reward_params().a_max()) {
    return (1.0 - acc / params_->vru_reward_params().a_max()) *
           params_->vru_reward_params().reward_max();
  } else if (acc < 0 && acc > params_->vru_reward_params().a_min()) {
    const double abs_ratio =
        std::abs(acc / params_->vru_reward_params().a_min());
    return (1.0 - abs_ratio) * params_->vru_reward_params().reward_max();
  }
  return 0.0;
}

double MCTSSpatiotemporal::CalculateJerkReward(const MCTSNodeConstPtr& node) {
  if (nullptr == node || nullptr == params_) {
    return 0.0;
  }
  if (ad_e2e::planning::math::Double::Compare(
          params_->vru_reward_params().jerk_max(), 0) ==
      ad_e2e::planning::math::Double::CompareType::EQUAL) {
    return 0.0;
  }
  const double jerk = node->state.leader_status.act_jerk;
  const double jerk_abs = std::abs(jerk);
  if (ad_e2e::planning::math::Double::Compare(
          jerk_abs, std::abs(params_->vru_reward_params().jerk_max())) !=
      ad_e2e::planning::math::Double::CompareType::GREATER) {
    return (1.0 - jerk_abs / params_->vru_reward_params().jerk_max()) *
           params_->vru_reward_params().reward_max();
  }
  return 0.0;
}

double MCTSSpatiotemporal::CalculateActionSmoothReward(
    const MCTSNodeConstPtr& node) {
  if (nullptr == node || nullptr == node->GetParent()) {
    return 0.0;
  }
  const double jerk_curr = node->state.leader_status.act_jerk;
  const double jerk_parent = node->GetParent()->state.leader_status.act_jerk;
  const double jerk_delta = jerk_curr - jerk_parent;
  return std::exp(-params_->vru_reward_params().action_coeff() *
                  std::abs(jerk_delta)) *
         params_->vru_reward_params().reward_max();
}

double MCTSSpatiotemporal::CalculateEndReward(const MCTSNodeConstPtr& node) {
  if (nullptr == node || nullptr == params_) {
    return 0.0;
  }
  if (node->is_leaf) {
    if (node->state.leader_status.v <
            params_->vru_reward_params().threshold_parking() &&
        node->state.follower_status.v <
            params_->vru_reward_params().threshold_parking()) {
      return params_->vru_reward_params().parking_penalty();
    }
  }
  return 0.0;
}

void MCTSSpatiotemporal::FindIntersectionAndCalculate(
    const double follower_x, const double follower_y,
    const double follower_heading, const double follower_speed,
    const double leader_x, const double leader_y, const double leader_speed,
    IntersectionResult& intersection_result) {
  if (nullptr == leader_path_ || leader_path_->size() < 2) {
    intersection_result.has_intersection = false;
    return;
  }
  const double ray_len = 1000.0;
  Vec2d follower_pos(follower_x, follower_y);
  Vec2d leader_pos(leader_x, leader_y);
  Vec2d follower_unit_direct = Vec2d::FastUnitFromAngle(follower_heading);
  Segment2d follower_segment(ray_len, follower_pos, follower_unit_direct);
  Vec2d intersect_point;
  for (size_t i = 0; i + 1 < leader_path_->size(); ++i) {
    const auto leader_segment_start = leader_path_->at(i);
    const auto leader_segment_end = leader_path_->at(i + 1);
    Segment2d segment(Vec2d(leader_segment_start.x(), leader_segment_start.y()),
                      Vec2d(leader_segment_end.x(), leader_segment_end.y()));
    if (segment.GetIntersect(follower_segment, &intersect_point)) {
      intersection_result.has_intersection = true;
      intersection_result.dis_leader = intersect_point.DistanceTo(leader_pos);
      intersection_result.dis_follower =
          intersect_point.DistanceTo(follower_pos);
      intersection_result.t_leader_arrive =
          (leader_speed > 0) ? intersection_result.dis_leader / leader_speed
                             : 0;
      intersection_result.t_follower_arrive =
          (follower_speed > 0)
              ? intersection_result.dis_follower / follower_speed
              : 0;
      break;
    }
  }
}

}  // namespace e2e_noa::planning
