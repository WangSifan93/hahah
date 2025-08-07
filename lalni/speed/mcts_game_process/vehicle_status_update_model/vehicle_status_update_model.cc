#include "vehicle_status_update_model.h"

#include "vehicle_simulation_model.h"

namespace e2e_noa {
namespace planning {

namespace {
constexpr double kEpsilon = 1e-3;
const std::vector<double> follower_actions = {-2.0, -1.0, 0.0, 1.0, 2.0};

double CalculateSafetyCost(const double obj_ttc, const double ttc_min,
                           const double ttc_max, const double cost_max) {
  return std::min(1.0,
                  std::max(0.0, (ttc_max - obj_ttc) /
                                    std::max(ttc_max - ttc_min, kEpsilon))) *
         cost_max;
}

double CalculateEfficiencyCost(const double obj_time, const double time_expect,

                               const double cost_max) {
  return obj_time < time_expect
             ? 0.0
             : std::min(1.0, (obj_time - time_expect) /
                                 std::max(time_expect, kEpsilon)) *
                   cost_max;
}

double CalculateAccelerationCost(const double acc, const double a_min,
                                 const double a_max, const double cost_max) {
  if (acc < 0.0) {
    return std::min(1.0, acc / std::min(a_min, -kEpsilon)) * cost_max;
  } else {
    return std::min(1.0, acc / std::max(a_max, kEpsilon)) * cost_max;
  }
}

double CalculateJerkCost(const double jerk, const double jerk_max,
                         const double cost_max) {
  return std::min(1.0, std::abs(jerk) / std::max(jerk_max, kEpsilon)) *
         cost_max;
}

double CalculateHeuristicCost(const double obj_time, const double leader_time,
                              const double obj_acc, const double action,
                              const double jerk_max, const double a_min,
                              const double cost_max) {
  if (obj_acc < -1.0 && obj_time < leader_time) {
    return std::min(1.0, obj_acc / std::min(a_min, -kEpsilon)) * cost_max;
  }

  if (leader_time < obj_time && leader_time + 0.5 > obj_time && action > -0.1) {
    return std::min(1.0, (action) / std::max(jerk_max, kEpsilon)) * cost_max;
  }
  return 0.0;
}

double CalculateFusedCost(const double tau, const std::vector<double> &costs) {
  double cost = 0.0;
  std::vector<double> normalized_costs;
  double sum_normalized_costs = 0.0;
  for (const auto &cost : costs) {
    normalized_costs.emplace_back(std::exp(-cost / std::max(tau, kEpsilon)));
    sum_normalized_costs += normalized_costs.back();
  }
  for (int i = 0; i < costs.size() && i < normalized_costs.size(); ++i) {
    cost += normalized_costs[i] / std::max(sum_normalized_costs, kEpsilon) *
            costs[i];
  }
  return cost;
}

void FindAllFollowerStates(
    const SpeedPlanningParamsProto::VehicleStatusUpdateModelParamsProto &params,
    const MCTSNodeConstPtr &parent, const double time_step,
    const double leader_time, VehicleSimulationModel &follower_model,
    std::vector<std::pair<AgentStatus, double>> &follower_states,
    double &ttc_min, double &ttc_max) {
  if (nullptr == parent) {
    return;
  }
  for (const auto &follower_action : follower_actions) {
    const double follower_a = parent->state.follower_status.a + follower_action;
    if (follower_a > params.a_max() * 1.5 || follower_a < params.a_min()) {
      continue;
    }
    follower_model.SetInitialState(parent->state.follower_status.s,
                                   parent->state.follower_status.v, follower_a);
    AgentStatus follower_status;
    follower_status.id = parent->state.follower_status.id;
    follower_status.s = follower_model.ComputeDistance(time_step);
    follower_status.v = follower_model.ComputeVelocity(time_step);
    follower_status.a = follower_a;
    follower_status.act_jerk = follower_action;
    follower_status.remain_dis =
        parent->state.follower_status.remain_dis -
        (follower_status.s - parent->state.follower_status.s);
    const double follow_time = follower_model.ComputeArriveTime(
        follower_status.remain_dis + follower_status.s);
    const double ttc = std::fabs(follow_time - leader_time);
    follower_states.emplace_back(std::make_pair(follower_status, follow_time));
    ttc_min = std::min(ttc_min, ttc);
    ttc_max = std::max(ttc_max, ttc);
  }
}

double CalculateFollowerCost(
    const SpeedPlanningParamsProto::VehicleStatusUpdateModelParamsProto &params,
    const AgentStatus &follower_status, const double obj_time,
    const double leader_time, const double ttc_min, const double ttc_max,
    const double follower_cur_v, const std::shared_ptr<LonDebug> &lon_debug) {
  std::vector<double> costs;

  const double safety_cost = CalculateSafetyCost(
      std::fabs(obj_time - leader_time), ttc_min, ttc_max, params.cost_max());
  costs.emplace_back(safety_cost);

  const double time_max = std::min(follower_cur_v * 1.3, 6.0);
  const double time_expect =
      follower_status.remain_dis / std::max(follower_cur_v * 0.6, time_max);
  const double eff_cost =
      CalculateEfficiencyCost(obj_time, time_expect, params.cost_max());
  costs.emplace_back(eff_cost);

  const double acc_cost = CalculateAccelerationCost(
      follower_status.a, params.a_min(), params.a_max(), params.cost_max());
  costs.emplace_back(acc_cost);

  const double jerk_cost = CalculateJerkCost(
      follower_status.act_jerk, params.jerk_max(), params.cost_max());
  costs.emplace_back(jerk_cost);

  const double heu_cost = CalculateHeuristicCost(
      obj_time, leader_time, follower_status.a, follower_status.act_jerk,
      params.jerk_max(), params.a_min(), params.cost_max());
  costs.emplace_back(heu_cost);

  const double total_cost = CalculateFusedCost(params.temperature(), costs);

  if (lon_debug != nullptr) {
    lon_debug->leader_time = leader_time;
    auto &cost_infos = lon_debug->cost_debugs.emplace_back();
    cost_infos.action = follower_status.act_jerk;
    cost_infos.acc = follower_status.a;
    cost_infos.cost_safe = safety_cost;
    cost_infos.cost_eff = eff_cost;
    cost_infos.cost_acc = acc_cost;
    cost_infos.cost_jerk = jerk_cost;
    cost_infos.cost_heu = heu_cost;
    cost_infos.total_cost = total_cost;
    cost_infos.follower_time = obj_time;
    cost_infos.time_expect = time_expect;
    cost_infos.ttc_max = ttc_max;
    cost_infos.ttc_min = ttc_min;
  }
  return total_cost;
}

}  // namespace

bool UpdateFollowerState(
    const MCTSNodeConstPtr &parent, const double leader_time,
    const double time_step, const double follower_cur_v,
    const SpeedPlanningParamsProto::VehicleStatusUpdateModelParamsProto &params,
    VehicleSimulationModel &follower_model, NodeState &state,
    const std::shared_ptr<LonDebug> &lon_debug) {
  if (nullptr == parent) {
    return false;
  }
  double ttc_min = std::numeric_limits<double>::max();
  double ttc_max = -std::numeric_limits<double>::max();
  std::vector<std::pair<AgentStatus, double>> follower_states;
  follower_states.reserve(follower_actions.size());
  FindAllFollowerStates(params, parent, time_step, leader_time, follower_model,
                        follower_states, ttc_min, ttc_max);

  int best_index = -1;
  double cost_min = std::numeric_limits<double>::max();
  for (int i = 0; i < follower_states.size(); ++i) {
    const double cost = CalculateFollowerCost(
        params, follower_states[i].first, follower_states[i].second,
        leader_time, ttc_min, ttc_max, follower_cur_v, lon_debug);
    if (cost < cost_min) {
      best_index = i;
      cost_min = cost;
    }
  }
  if (-1 == best_index || best_index + 1 > follower_states.size()) {
    return false;
  }
  state.follower_status = follower_states[best_index].first;

  return true;
}

}  // namespace planning
}  // namespace e2e_noa
