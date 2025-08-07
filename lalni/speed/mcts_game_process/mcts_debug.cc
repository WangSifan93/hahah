#include "mcts_debug.h"

#include <fstream>

#include "nlohmann/nlohmann/json.hpp"

namespace e2e_noa {
namespace planning {

constexpr int MAX_LAYER = 10;
namespace {
double ConfigureDecimalPrecision(const double num) {
  return std::round(num * 1000) / 1000;
}

void AgentStatusToJson(const AgentStatus &as, nlohmann::json &agent_json) {
  agent_json["id"] = as.id;
  agent_json["s"] = ConfigureDecimalPrecision(as.s);
  agent_json["v"] = ConfigureDecimalPrecision(as.v);
  agent_json["a"] = ConfigureDecimalPrecision(as.a);
  agent_json["act_jerk"] = ConfigureDecimalPrecision(as.act_jerk);
  agent_json["remain_dis"] = ConfigureDecimalPrecision(as.remain_dis);
  agent_json["x"] = ConfigureDecimalPrecision(as.x);
  agent_json["y"] = ConfigureDecimalPrecision(as.y);
  agent_json["angular_v"] = ConfigureDecimalPrecision(as.angular_v);
  agent_json["act_angular_a"] = ConfigureDecimalPrecision(as.act_angular_a);
  agent_json["heading"] = ConfigureDecimalPrecision(as.heading);
}

void NodeStateToJson(const NodeState &ns, nlohmann::json &state_json) {
  AgentStatusToJson(ns.leader_status, state_json["leader_status"]);
  AgentStatusToJson(ns.follower_status, state_json["follower_status"]);
}

void LonDebugToJson(const LonDebug &lon_debug, nlohmann::json &lon_debug_json) {
  lon_debug_json["debug_info"] = lon_debug.debug_info;
  lon_debug_json["reward"]["total_reward"] =
      ConfigureDecimalPrecision(lon_debug.total_reward);
  lon_debug_json["reward"]["reward_safe"] =
      ConfigureDecimalPrecision(lon_debug.reward_safe);
  lon_debug_json["reward"]["reward_eff"] =
      ConfigureDecimalPrecision(lon_debug.reward_eff);
  lon_debug_json["reward"]["reward_acc"] =
      ConfigureDecimalPrecision(lon_debug.reward_acc);
  lon_debug_json["reward"]["reward_jerk"] =
      ConfigureDecimalPrecision(lon_debug.reward_jerk);
  lon_debug_json["reward"]["reward_act"] =
      ConfigureDecimalPrecision(lon_debug.reward_act);
  lon_debug_json["reward"]["reward_heu"] =
      ConfigureDecimalPrecision(lon_debug.reward_heu);

  lon_debug_json["cost"]["leader_time"] =
      ConfigureDecimalPrecision(lon_debug.leader_time);
  for (const auto &cost_info : lon_debug.cost_debugs) {
    const auto action =
        std::to_string(ConfigureDecimalPrecision(cost_info.action));
    lon_debug_json["cost"][action]["follower_time"] =
        ConfigureDecimalPrecision(cost_info.follower_time);
    lon_debug_json["cost"][action]["time_expect"] =
        ConfigureDecimalPrecision(cost_info.time_expect);
    lon_debug_json["cost"][action]["ttc_max"] =
        ConfigureDecimalPrecision(cost_info.ttc_max);
    lon_debug_json["cost"][action]["ttc_min"] =
        ConfigureDecimalPrecision(cost_info.ttc_min);

    lon_debug_json["cost"][action]["acc"] =
        ConfigureDecimalPrecision(cost_info.acc);
    lon_debug_json["cost"][action]["cost_safe"] =
        ConfigureDecimalPrecision(cost_info.cost_safe);
    lon_debug_json["cost"][action]["cost_eff"] =
        ConfigureDecimalPrecision(cost_info.cost_eff);
    lon_debug_json["cost"][action]["cost_acc"] =
        ConfigureDecimalPrecision(cost_info.cost_acc);
    lon_debug_json["cost"][action]["cost_jerk"] =
        ConfigureDecimalPrecision(cost_info.cost_jerk);
    lon_debug_json["cost"][action]["cost_heu"] =
        ConfigureDecimalPrecision(cost_info.cost_heu);
    lon_debug_json["cost"][action]["total_cost"] =
        ConfigureDecimalPrecision(cost_info.total_cost);
  }
}

void SerializeSptDebugData(const SptDebug &spt_debug,
                           nlohmann::json &spt_debug_json) {
  spt_debug_json["debug_info"] = spt_debug.debug_info;
  spt_debug_json["reward"]["total_reward"] =
      ConfigureDecimalPrecision(spt_debug.total_reward);
  spt_debug_json["reward"]["reward_self"] =
      ConfigureDecimalPrecision(spt_debug.reward_self);
  spt_debug_json["reward"]["reward_other"] =
      ConfigureDecimalPrecision(spt_debug.reward_other);
  spt_debug_json["reward"]["reward_safe"] =
      ConfigureDecimalPrecision(spt_debug.reward_safe);
  spt_debug_json["reward"]["reward_eff"] =
      ConfigureDecimalPrecision(spt_debug.reward_eff);
  spt_debug_json["reward"]["reward_acc"] =
      ConfigureDecimalPrecision(spt_debug.reward_acc);
  spt_debug_json["reward"]["reward_jerk"] =
      ConfigureDecimalPrecision(spt_debug.reward_jerk);
  spt_debug_json["reward"]["reward_act"] =
      ConfigureDecimalPrecision(spt_debug.reward_act);
  spt_debug_json["reward_self_step"] =
      ConfigureDecimalPrecision(spt_debug.reward_self_step);
  spt_debug_json["reward"]["reward_self_end"] =
      ConfigureDecimalPrecision(spt_debug.reward_self_end);
  spt_debug_json["reward"]["reward_follower_delta_v"] =
      ConfigureDecimalPrecision(spt_debug.reward_follower_delta_v);
  spt_debug_json["reward"]["reward_follower_delta_heading"] =
      ConfigureDecimalPrecision(spt_debug.reward_follower_delta_heading);
  spt_debug_json["reward"]["reward_follower_v"] =
      ConfigureDecimalPrecision(spt_debug.reward_follower_v);
  nlohmann::json vru_action_reward_array = nlohmann::json::array();
  for (const auto &cost_info : spt_debug.vru_action_and_costs) {
    vru_action_reward_array.push_back(cost_info.to_json());
  }
  spt_debug_json["vru"] = vru_action_reward_array;
}

void MCTSNodeToJson(const MCTSNode &node, nlohmann::json &node_json) {
  NodeStateToJson(node.state, node_json["state"]);
  node_json["visits"] = node.visits;
  node_json["is_leaf"] = node.is_leaf;
  node_json["current_time"] = ConfigureDecimalPrecision(node.current_time);
  node_json["step_reward"] = node.step_reward;
  node_json["reward"] = node.reward;
  node_json["parent_id"] =
      nullptr == node.GetParent() ? -1 : node.GetParent()->id;
  if (node.lon_debug != nullptr) {
    LonDebugToJson(*node.lon_debug, node_json["lon_debug"]);
  }
  if (node.spt_debug != nullptr) {
    SerializeSptDebugData(*node.spt_debug, node_json["spt_debug"]);
  }
}

void MCTSSimpleNodeToJson(const MCTSNode &node, nlohmann::json &node_json) {
  NodeStateToJson(node.state, node_json["state"]);
  node_json["visits"] = node.visits;
  node_json["is_leaf"] = node.is_leaf;
  node_json["current_time"] = ConfigureDecimalPrecision(node.current_time);
  node_json["step_reward"] = ConfigureDecimalPrecision(node.step_reward);
  node_json["reward"] = ConfigureDecimalPrecision(node.reward);
}

void WalkTreeStructure(const MCTSNode &node, nlohmann::json &node_json_map) {
  if (node.visits < 1) {
    return;
  }
  MCTSNodeToJson(node, node_json_map[std::to_string(node.id)]);
  for (const auto &child : node.children) {
    if (nullptr == child || child->visits < 1) {
      continue;
    }
    WalkTreeStructure(*child, node_json_map);
  }
}

void BuildTreeWithIdsToJson(const MCTSNode &node, nlohmann::json &tree_json) {
  std::vector<nlohmann::json> children;
  if (node.visits < 1) {
    return;
  }
  for (const auto &child : node.children) {
    if (nullptr == child || child->visits < 1) {
      continue;
    }
    auto &child_json = children.emplace_back();
    BuildTreeWithIdsToJson(*child, child_json);
  }
  tree_json["children"] = std::move(children);
  tree_json["id"] = node.id;
}

void SearchDebugToJson(const std::map<int, SearchDebug> &search_debug_map,
                       nlohmann::json &search_json) {
  nlohmann::json debug_entries;
  for (const auto &[node_id, debug_data] : search_debug_map) {
    debug_entries[std::to_string(node_id)] = {
        {"reward_debug", debug_data.reward_debug},
        {"visits_debug", debug_data.visits_debug}};
  }
  search_json["search_debug"] = debug_entries;
}

void BestPathToJson(const std::vector<MCTSNodeConstPtr> &best_path,
                    nlohmann::json &path_json) {
  if (!path_json.contains("best_path")) {
    path_json["best_path"] = nlohmann::json::array();
  }
  for (auto best_node : best_path) {
    if (nullptr == best_node) {
      continue;
    }
    path_json["best_path"].push_back(best_node->id);
  }
}

void BestPathInfoToJson(const std::vector<MCTSNodeConstPtr> &best_path,
                        nlohmann::json &path_json) {
  for (auto best_node : best_path) {
    if (nullptr == best_node) {
      continue;
    }
    MCTSSimpleNodeToJson(*best_node, path_json[std::to_string(best_node->id)]);
  }
}

void LeafDebugToJson(const std::set<int> &leaf_debug,
                     nlohmann::json &leaf_json) {
  leaf_json["leaf_debug"] = nlohmann::json::array();
  for (int node_id : leaf_debug) {
    leaf_json["leaf_debug"].push_back(node_id);
  }
}

void TrajIDsDebugToJson(const std::vector<std::string> &traj_ids,
                        nlohmann::json &traj_json) {
  traj_json["traj_ids"] = nlohmann::json::array();
  for (const auto &traj_id : traj_ids) {
    traj_json["traj_ids"].push_back(traj_id);
  }
}

void STGraphDebugToJson(const StBoundaryWithDecision &st_boundary_with_decision,
                        nlohmann::json &st_graph_json) {
  auto get_log_points = [](const StBoundary *const st_boundary) {
    std::vector<StPoint> log_st_points;
    if (!st_boundary) {
      return log_st_points;
    }
    const auto &lower_points = st_boundary->lower_points();
    const auto &upper_points = st_boundary->upper_points();
    if (lower_points.empty() || upper_points.empty()) {
      return log_st_points;
    }

    for (int i = 0; i < lower_points.size(); i++) {
      log_st_points.push_back(lower_points.at(i));
    }
    for (int i = lower_points.size() - 1; i >= 0; i--) {
      log_st_points.push_back(upper_points.at(i));
    }
    return log_st_points;
  };
  const auto &raw_st_boundary = st_boundary_with_decision.raw_st_boundary();
  const auto &st_boundary = st_boundary_with_decision.st_boundary();
  std::string decision_type = "";
  switch (st_boundary_with_decision.decision_type()) {
    case StBoundaryProto::FOLLOW: {
      decision_type = "_F";
      break;
    }
    case StBoundaryProto::YIELD: {
      decision_type = "_Y";
      break;
    }
    case StBoundaryProto::OVERTAKE: {
      decision_type = "_O";
      break;
    }
    case StBoundaryProto::IGNORE: {
      decision_type = "_I";
      break;
    }
    case StBoundaryProto::UNKNOWN: {
      decision_type = "_U";
      break;
    }
    default:
      break;
  }

  std::string ignore_reason = "";
  if (st_boundary_with_decision.decision_type() == StBoundaryProto::IGNORE) {
    ignore_reason = "_" + StBoundaryProto::IgnoreReason_Name(
                              st_boundary_with_decision.ignore_reason());
  }

  const auto log_raw_st_points = get_log_points(raw_st_boundary);
  const auto log_st_points = get_log_points(st_boundary);
  if (!log_raw_st_points.empty()) {
    std::vector<double> xs;
    std::vector<double> ys;
    for (auto point : log_raw_st_points) {
      xs.emplace_back(point.t());
      ys.emplace_back(point.s());
    }
    const std::string raw_st_boundary_id =
        "st_raw_" +
        std::string(
            {raw_st_boundary->id().data(), raw_st_boundary->id().size()}) +
        decision_type + ignore_reason;
    st_graph_json["raw_st"] = {
        {"id", raw_st_boundary_id}, {"xs", xs}, {"ys", ys}};
  }

  if (!log_st_points.empty()) {
    std::vector<double> xs;
    std::vector<double> ys;
    for (auto point : log_st_points) {
      xs.emplace_back(point.t());
      ys.emplace_back(point.s());
    }
    const std::string st_boundary_id =
        "st_" +
        std::string({st_boundary->id().data(), st_boundary->id().size()}) +
        decision_type + ignore_reason;
    st_graph_json["st"] = {{"id", st_boundary_id}, {"xs", xs}, {"ys", ys}};
  }
}
void STGraphsDebugToJson(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const std::string &traj_id, nlohmann::json &st_graphs_json) {
  const std::string raw_id = traj_id + "|raw";
  const std::string modify_id = traj_id + "|m";
  for (const auto &boundary_with_decision : st_boundaries_with_decision) {
    const auto &cur_id = boundary_with_decision.st_boundary()->id();
    if (cur_id == traj_id || cur_id == raw_id) {
      STGraphDebugToJson(boundary_with_decision, st_graphs_json["original_st"]);
    } else if (cur_id == modify_id) {
      STGraphDebugToJson(boundary_with_decision, st_graphs_json["modify_st"]);
    }
  }
}

void SaveTreeToJson(
    const int &frame_id,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const std::vector<MCTSInteractiveResult> &interactive_results,
    const std::string &filename, const double is_online) {
  if (interactive_results.empty()) {
    return;
  }
  nlohmann::json json_file;
  json_file["frame_id"] = frame_id;
  bool has_valid_result = false;
  std::vector<std::string> traj_ids;
  for (const auto &interactive_result : interactive_results) {
    if (interactive_result.mcts_result.empty() ||
        interactive_result.mcts_result.front() == nullptr ||
        interactive_result.mcts_result.front()->visits < 1) {
      continue;
    }
    const auto &traj_id = interactive_result.traj_id;
    const auto &debug_info = interactive_result.debug_info;
    auto &traj_json = json_file[traj_id];
    traj_json["is_valid"] = interactive_result.is_valid;
    traj_json["scenario_info"] = interactive_result.scenario_info;
    STGraphsDebugToJson(st_boundaries_with_decision, traj_id, traj_json);
    SearchDebugToJson(debug_info.search_debug, traj_json);
    LeafDebugToJson(debug_info.leaf_debug, traj_json);
    BestPathToJson(interactive_result.mcts_result, traj_json);
    if (!interactive_result.mcts_result.empty() &&
        interactive_result.mcts_result.front() != nullptr) {
      const auto &root = interactive_result.mcts_result.front();
      BuildTreeWithIdsToJson(*root, traj_json["tree"]);
      WalkTreeStructure(*root, traj_json);
    }
    traj_ids.emplace_back(traj_id);
    has_valid_result = true;
  }
  if (!has_valid_result) {
    return;
  }
  TrajIDsDebugToJson(traj_ids, json_file);

  if (is_online) {
    std::string json_str = json_file.dump(4);
    Log2FG::LogDataV0("MCTS", json_str);
  } else {
    std::ofstream file(filename);
    if (file.is_open()) {
      file << json_file.dump(4);
      file.close();
    }
  }
}

void SaveSimpleTreeToJson(
    const std::vector<MCTSInteractiveResult> &interactive_results) {
  if (interactive_results.empty()) {
    return;
  }
  nlohmann::json json_file;
  std::vector<std::string> traj_ids;
  for (const auto &interactive_result : interactive_results) {
    const auto &traj_id = interactive_result.traj_id;
    const auto &debug_info = interactive_result.debug_info;
    auto &traj_json = json_file[traj_id];
    traj_json["is_valid"] = interactive_result.is_valid;
    traj_json["scenario_info"] = interactive_result.scenario_info;
    LeafDebugToJson(debug_info.leaf_debug, traj_json);
    BestPathInfoToJson(interactive_result.mcts_result, traj_json);
    traj_ids.emplace_back(traj_id);
  }
  TrajIDsDebugToJson(traj_ids, json_file);

  std::string json_str = json_file.dump(4);
  Log2FG::LogDataV0("SimpleMCTS", json_str);
}
}  // namespace

void OnlineMCTSSimpleDebug(
    const std::vector<MCTSInteractiveResult> &interactive_results) {
  SaveSimpleTreeToJson(interactive_results);
}

void OnlineMCTSDebug(
    const int plan_id, const uint64_t seq_num,
    const std::vector<MCTSInteractiveResult> &interactive_results) {
  std::string file_name = "";
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  SaveTreeToJson(seq_num, st_boundaries_with_decision, interactive_results,
                 file_name, true);
}

void OfflineMCTSDebug(
    const int plan_id, const uint64_t seq_num,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const std::vector<MCTSInteractiveResult> &interactive_results) {
  std::string directory = "./debug";
  if (!std::filesystem::exists(directory)) {
    if (!std::filesystem::create_directories(directory)) {
      return;
    }
  }
  std::string file_name = directory + "/" + std::to_string(seq_num) + "_" +
                          std::to_string(plan_id) + ".json";
  SaveTreeToJson(seq_num, st_boundaries_with_decision, interactive_results,
                 file_name, false);
}

}  // namespace planning
}  // namespace e2e_noa
