
#ifndef PLANNER_SPEED_MCTS_DATA_TYPE_H_
#define PLANNER_SPEED_MCTS_DATA_TYPE_H_
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "nlohmann/nlohmann/json.hpp"
#include "object/spacetime_object_trajectory.h"
#include "speed/st_boundary.h"
#include "speed/st_boundary_with_decision.h"
#include "speed_planning_params.pb.h"

#define VRU_DEBUG (0)

namespace e2e_noa {
namespace planning {
struct LonCostDebug {
  double action = 0.0;
  double acc = 0.0;
  double cost_safe = 0.0;
  double cost_eff = 0.0;
  double cost_acc = 0.0;
  double cost_jerk = 0.0;
  double cost_heu = 0.0;
  double total_cost = 0.0;
  double follower_time = 0.0;
  double time_expect = 0.0;
  double ttc_max = 0.0;
  double ttc_min = 0.0;
};
struct LonDebug {
  std::string debug_info = "";
  double total_reward = 0.0;
  double reward_safe = 0.0;
  double reward_eff = 0.0;
  double reward_acc = 0.0;
  double reward_jerk = 0.0;
  double reward_act = 0.0;
  double reward_heu = 0.0;
  double leader_time = 0.0;
  std::vector<LonCostDebug> cost_debugs;
};

struct ActionandCost {
  double jerk = 0.0;
  double angular_a = 0.0;
  double tau_min = 0.0;
  double SafetyCost = 0.0;
  double GoalCost = 0.0;
  double SpeedCost = 0.0;
  double AccelCost = 0.0;
  double JerkCost = 0.0;
  double AngSpeedCost = 0.0;
  double AngAccelCost = 0.0;
  double TotalCost = 0.0;

  nlohmann::json to_json() const {
    return {{"jerk", jerk},
            {"angular_a", angular_a},
            {"tau_min", tau_min},
            {"SafetyCost", SafetyCost},
            {"GoalCost", GoalCost},
            {"SpeedCost", SpeedCost},
            {"AccelCost", AccelCost},
            {"JerkCost", JerkCost},
            {"AngSpeedCost", AngSpeedCost},
            {"AngAccelCost", AngAccelCost},
            {"TotalCost", TotalCost}};
  }
};

struct SptDebug {
  std::string debug_info = "";
  double total_reward = 0.0;
  double reward_self = 0.0;
  double reward_other = 0.0;
  double reward_safe = 0.0;
  double reward_eff = 0.0;
  double reward_acc = 0.0;
  double reward_jerk = 0.0;
  double reward_act = 0.0;
  double reward_self_step = 0.0;
  double reward_self_end = 0.0;
  double reward_follower_delta_v = 0.0;
  double reward_follower_delta_heading = 0.0;
  double reward_follower_v = 0.0;
  std::vector<ActionandCost> vru_action_and_costs;
};

struct AgentStatus {
  std::string id = "";
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  double act_jerk = 0.0;
  double remain_dis = 0.0;

  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double angular_v = 0.0;
  double act_angular_a = 0.0;
  AgentStatus() = default;
  AgentStatus(const std::string &id, const double s, const double v,
              const double a, const double act_jerk, const double remain_dis)
      : id(id),
        s(s),
        v(v),
        a(a),
        act_jerk(act_jerk),
        remain_dis(remain_dis),
        x(0.0),
        y(0.0),
        heading(0.0),
        angular_v(0.0),
        act_angular_a(0.0) {}

  AgentStatus(const std::string &id, const double x, const double y,
              const double heading, const double v, const double a,
              const double angular_v, const double act_jerk,
              const double act_angular_a)
      : id(id),
        s(0.0),
        v(v),
        a(a),
        act_jerk(act_jerk),
        remain_dis(0.0),
        x(x),
        y(y),
        heading(heading),
        angular_v(angular_v),
        act_angular_a(act_angular_a) {}
};

struct NodeState {
  AgentStatus leader_status;
  AgentStatus follower_status;
  double leader_arrive_time = 0.0;
  NodeState() = default;
};

struct MCTSNode {
  NodeState state;
  std::weak_ptr<MCTSNode> parent;
  std::vector<std::shared_ptr<MCTSNode>> children;
  int id = -1;
  int visits = 0;
  bool is_leaf = false;
  double current_time = 0.0;
  double step_reward = 0.0;
  double reward = 0.0;
  std::shared_ptr<LonDebug> lon_debug = nullptr;
  std::shared_ptr<SptDebug> spt_debug = nullptr;
  std::optional<std::vector<double>> reward_lists = std::nullopt;
  std::optional<std::vector<int>> visits_lists = std::nullopt;

  MCTSNode(const NodeState &s, std::shared_ptr<MCTSNode> p = nullptr)
      : state(s), parent(p) {}

  std::shared_ptr<const MCTSNode> GetParent() const { return parent.lock(); }
  std::shared_ptr<MCTSNode> GetMutableParent() const { return parent.lock(); }
};

struct SearchDebug {
  std::vector<double> reward_debug;
  std::vector<int> visits_debug;
};

struct MCTSDebugInfo {
  std::map<int, SearchDebug> search_debug;
  std::set<int> leaf_debug;
};

struct MCTSInteractiveResult {
  std::string traj_id;
  std::vector<std::shared_ptr<const MCTSNode>> mcts_result;
  const SpacetimeObjectTrajectory *spacetime_obj = nullptr;
  const StBoundary *st_boundary = nullptr;
  bool is_valid = true;
  bool is_hrvo = false;
  std::string scenario_info = "";
  MCTSDebugInfo debug_info;
};

struct InteractiveInput {
  bool is_hrvo = false;
  StBoundaryWithDecision *boundary_with_decision = nullptr;
  std::vector<StBoundaryWithDecision *> other_vehicles;
};

using MCTSNodePtr = std::shared_ptr<MCTSNode>;
using MCTSNodeConstPtr = std::shared_ptr<const MCTSNode>;
}  // namespace planning
}  // namespace e2e_noa
#endif
