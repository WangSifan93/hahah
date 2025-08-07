#include "mcts_longitudinal.h"

#include "vehicle_status_update_model/vehicle_status_update_model.h"

namespace e2e_noa {
namespace planning {

constexpr double kEpsilon = 1e-3;
constexpr double kRadio = 0.8;
bool MCTSLongitudinal::PrePruning(const MCTSNodePtr& node,
                                  const double leader_action) {
  if (nullptr == node || nullptr == params_) {
    return true;
  }
  const double cur_a = node->state.leader_status.a;
  const double cur_v = node->state.leader_status.v;
  const double new_a = cur_a + leader_action;
  if (new_a > params_->a_max() || new_a < params_->a_min()) {
    return true;
  }
  if (ad_e2e::planning::math::Double::Compare(cur_v, leader_max_v_) ==
          ad_e2e::planning::math::Double::CompareType::GREATER &&
      new_a > 0.0) {
    return true;
  }

  return false;
}

void MCTSLongitudinal::CreateChildNode(const MCTSNodePtr& parent,
                                       const double leader_action,
                                       MCTSNodePtr& child) {
  if (nullptr == parent || nullptr == params_ ||
      !params_->has_vehicle_status_update_model_params()) {
    return;
  }
  NodeState state;
  UpdateLeaderState(parent, leader_action, state);
  child = std::make_shared<MCTSNode>(state, parent);
  child->current_time = parent->current_time + time_step_;
}

void MCTSLongitudinal::UpdateChildNode(const MCTSNodeConstPtr& parent,
                                       const MCTSNodePtr& child) {
  if (nullptr == parent || nullptr == params_ ||
      !params_->has_vehicle_status_update_model_params()) {
    return;
  }
  NodeState& state = child->state;
  if (enable_lon_debug_ && nullptr == child->lon_debug) {
    child->lon_debug = std::make_shared<LonDebug>();
  }
  if (!UpdateFollowerState(parent, child->state.leader_arrive_time, time_step_,
                           follower_cur_v_,
                           params_->vehicle_status_update_model_params(),
                           follower_model_, state, child->lon_debug)) {
    return;
  }
  child->is_leaf = IsLeaf(child);
}

bool MCTSLongitudinal::IsLeaf(const MCTSNodeConstPtr& node) {
  if (nullptr == node) {
    return false;
  }
  if (ad_e2e::planning::math::Double::Compare(
          node->state.leader_status.remain_dis, 0.0) ==
          ad_e2e::planning::math::Double::CompareType::LESS ||
      ad_e2e::planning::math::Double::Compare(
          node->state.follower_status.remain_dis, 0.0) ==
          ad_e2e::planning::math::Double::CompareType::LESS ||
      (ad_e2e::planning::math::Double::Compare(node->state.leader_status.v,
                                               0.0) ==
           ad_e2e::planning::math::Double::CompareType::EQUAL &&
       (ad_e2e::planning::math::Double::Compare(node->state.follower_status.v,
                                                0.0) ==
        ad_e2e::planning::math::Double::CompareType::EQUAL))) {
    return true;
  }
  return false;
}

void MCTSLongitudinal::UpdateLeaderState(const MCTSNodeConstPtr& parent,
                                         const double leader_action,
                                         NodeState& state) {
  if (nullptr == parent) {
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
}

void MCTSLongitudinal::UpdateNodeStepReward(const MCTSNodePtr& node) {
  if (nullptr == node || nullptr == params_) {
    return;
  }
  leader_model_.SetInitialState(node->state.leader_status.s,
                                node->state.leader_status.v,
                                node->state.leader_status.a);
  follower_model_.SetInitialState(node->state.follower_status.s,
                                  node->state.follower_status.v,
                                  node->state.follower_status.a);
  if (!leader_model_.IsInited() || !follower_model_.IsInited()) {
    return;
  }
  const double dis_leader = node->state.leader_status.remain_dis;
  const double dis_follower = node->state.follower_status.remain_dis;
  const double t_leader_arrive =
      leader_model_.ComputeArriveTime(dis_leader + node->state.leader_status.s);
  const double t_follower_arrive = follower_model_.ComputeArriveTime(
      dis_follower + node->state.follower_status.s);

  const double reward_safe =
      CalculateSafetyReward(node, t_leader_arrive, t_follower_arrive);
  const double reward_eff = CalculateEfficiencyReward(node, t_leader_arrive);
  const double reward_acc = CalculateAccelerationReward(node);
  const double reward_jerk = CalculateJerkReward(node);
  const double reward_act = CalculateActionSmoothReward(node);
  const double reward_heu =
      CalculateHeuristicsReward(node, t_leader_arrive, t_follower_arrive);

  node->step_reward = CalculateStepReward(reward_safe, reward_eff, reward_acc,
                                          reward_jerk, reward_act, reward_heu);

  if (enable_lon_debug_) {
    if (nullptr == node->lon_debug) {
      node->lon_debug = std::make_shared<LonDebug>();
    }
    node->lon_debug->total_reward = node->step_reward;
    node->lon_debug->reward_safe = reward_safe;
    node->lon_debug->reward_eff = reward_eff;
    node->lon_debug->reward_acc = reward_acc;
    node->lon_debug->reward_jerk = reward_jerk;
    node->lon_debug->reward_act = reward_act;
    node->lon_debug->reward_heu = reward_heu;
  }
}

void MCTSLongitudinal::InitLeaderModel(const double min_v, const double max_v,
                                       const double time_step,
                                       const double jerk) {
  leader_max_v_ = std::max(max_v, kEpsilon);
  time_step_ = time_step;
  leader_model_ = VehicleSimulationModel(min_v, max_v, time_step, jerk);
}

void MCTSLongitudinal::InitFollowerModel(const double min_v, const double max_v,
                                         const double time_step,
                                         const double jerk) {
  follower_model_ = VehicleSimulationModel(min_v, max_v, time_step, jerk);
}

double MCTSLongitudinal::CalculateStepReward(const double reward_safe,
                                             const double reward_eff,
                                             const double reward_acc,
                                             const double reward_jerk,
                                             const double reward_act,
                                             const double reward_heu) {
  return params_->w_safe() * reward_safe + params_->w_eff() * reward_eff +
         params_->w_acc() * reward_acc + params_->w_jerk() * reward_jerk +
         params_->w_act() * reward_act + params_->w_heu() * reward_heu;
}

double MCTSLongitudinal::CalculateSafetyReward(const MCTSNodeConstPtr& node,
                                               const double t_leader_arrive,
                                               const double t_follower_arrive) {
  if (nullptr == node) {
    return 0.0;
  }

  const double dis_leader = node->state.leader_status.remain_dis;
  const double dis_follower = node->state.follower_status.remain_dis;
  const double t_diff = t_leader_arrive - t_follower_arrive;
  const double v_leader = node->state.leader_status.v;
  const double v_follower = node->state.follower_status.v;

  if (node->is_leaf) {
    if (ad_e2e::planning::math::Double::Compare(
            dis_leader, params_->threshold_collision()) ==
            ad_e2e::planning::math::Double::CompareType::LESS &&
        ad_e2e::planning::math::Double::Compare(
            dis_follower, params_->threshold_collision()) ==
            ad_e2e::planning::math::Double::CompareType::LESS) {
      return params_->reward_min();
    }
    constexpr double v_threshold = 1e-1;
    if (ad_e2e::planning::math::Double::Compare(std::abs(v_leader - v_follower),
                                                v_threshold) ==
        ad_e2e::planning::math::Double::CompareType::LESS) {
      return params_->reward_max();
    }
  }

  if (ad_e2e::planning::math::Double::Compare(std::abs(t_diff),
                                              params_->t_diff_max()) ==
      ad_e2e::planning::math::Double::CompareType::LESS) {
    return (std::abs(t_diff) / params_->t_diff_max()) * params_->reward_max();
  } else {
    return params_->reward_max();
  }
  return 0.0;
}

double MCTSLongitudinal::CalculateEfficiencyReward(const MCTSNodeConstPtr& node,
                                                   const double t_arrive) {
  if (nullptr == node) {
    return 0.0;
  }
  const double dis_leader = node->state.leader_status.remain_dis;
  if (ad_e2e::planning::math::Double::Compare(params_->v_perf(), 0) ==
      ad_e2e::planning::math::Double::CompareType::EQUAL) {
    return 0.0;
  }
  const double t_expected =
      dis_leader /
      std::max(leader_cur_v_ * params_->radio_v_perf(), params_->v_perf());
  if (ad_e2e::planning::math::Double::Compare(t_arrive, t_expected) ==
          ad_e2e::planning::math::Double::CompareType::LESS ||

      ad_e2e::planning::math::Double::Compare(t_arrive, 0) ==
          ad_e2e::planning::math::Double::CompareType::EQUAL) {
    return params_->reward_max();
  }
  return std::min(1.0, (t_arrive / t_expected - 1.0)) * params_->reward_min();
};

double MCTSLongitudinal::CalculateAccelerationReward(
    const MCTSNodeConstPtr& node) {
  if (nullptr == node) {
    return 0.0;
  }
  const double acc = node->state.leader_status.a;

  if (ad_e2e::planning::math::Double::Compare(acc, 0) !=
          ad_e2e::planning::math::Double::CompareType::LESS &&
      ad_e2e::planning::math::Double::Compare(acc, params_->a_max()) ==
          ad_e2e::planning::math::Double::CompareType::LESS) {
    return (1.0 - acc / params_->a_max()) * params_->reward_max();
  } else if (ad_e2e::planning::math::Double::Compare(acc, 0) ==
                 ad_e2e::planning::math::Double::CompareType::LESS &&
             ad_e2e::planning::math::Double::Compare(acc, params_->a_min()) ==
                 ad_e2e::planning::math::Double::CompareType::GREATER) {
    const double abs_ratio = std::abs(acc / params_->a_min());
    return (1.0 - abs_ratio) * params_->reward_max();
  }
  return 0.0;
};

double MCTSLongitudinal::CalculateJerkReward(const MCTSNodeConstPtr& node) {
  if (nullptr == node) {
    return 0.0;
  }
  if (ad_e2e::planning::math::Double::Compare(params_->jerk_max(), 0) ==
      ad_e2e::planning::math::Double::CompareType::EQUAL) {
    return 0.0;
  }
  const double jerk = node->state.leader_status.act_jerk;
  const double jerk_abs = std::abs(jerk);
  if (ad_e2e::planning::math::Double::Compare(jerk_abs,
                                              std::abs(params_->jerk_max())) !=
      ad_e2e::planning::math::Double::CompareType::GREATER) {
    return (1.0 - jerk_abs / params_->jerk_max()) * params_->reward_max();
  }
  return 0.0;
};

double MCTSLongitudinal::CalculateActionSmoothReward(
    const MCTSNodeConstPtr& node) {
  if (nullptr == node || nullptr == node->GetParent() ||
      nullptr == node->GetParent()->GetParent()) {
    return 0.0;
  }
  const double jerk_curr = node->state.leader_status.act_jerk;
  const double jerk_parent = node->GetParent()->state.leader_status.act_jerk;
  const double jerk_delta = jerk_curr - jerk_parent;
  return std::exp(-params_->action_coeff() * std::abs(jerk_delta)) *
         params_->reward_max();
};

double MCTSLongitudinal::CalculateHeuristicsReward(
    const MCTSNodeConstPtr& node, const double t_leader_arrive,
    const double t_follower_arrive) {
  if (nullptr == node) {
    return 0.0;
  }
  const double dis_leader = node->state.leader_status.remain_dis;
  const double acc_leader = node->state.leader_status.a;

  if (dis_leader < params_->threshold_gamepoint()) {
    const double a_limit = acc_leader > 0 ? params_->a_max() : params_->a_min();
    const double reward =
        acc_leader > 0 ? params_->reward_max() : params_->reward_min();
    const double ratio = std::min(acc_leader / a_limit, 1.0);
    return ratio * reward;
  }

  if (t_leader_arrive < t_follower_arrive) {
    const double a_limit = acc_leader > 0 ? params_->a_max() : params_->a_min();
    const double reward =
        acc_leader > 0 ? params_->reward_max() : params_->reward_min();
    const double ratio = std::min(acc_leader / a_limit, 1.0);
    return ratio * reward;
  } else {
    const double a_limit = acc_leader > 0 ? params_->a_max() : params_->a_min();
    const double reward =
        acc_leader > 0 ? params_->reward_min() : params_->reward_max();
    const double ratio = std::min(acc_leader / a_limit, 1.0);
    return ratio * reward;
  }

  return 0.0;
};
}  // namespace planning
}  // namespace e2e_noa
