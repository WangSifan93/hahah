#include "mcts.h"

#include <fstream>

#include "nlohmann/nlohmann/json.hpp"

namespace e2e_noa {
namespace planning {

namespace {
constexpr int MAX_LAYER = 10;
void BuildLeafDebug(const MCTSNode &node, std::set<int> &leaf_debug) {
  if (!node.is_leaf) {
    leaf_debug.emplace(node.id);
  }
}

void BuildSearchDebug(const MCTSNode &node,
                      std::map<int, SearchDebug> &search_debug) {
  if (!node.children.empty()) {
    for (const auto &children : node.children) {
      if (nullptr == children) {
        continue;
      }
      search_debug[children->id].visits_debug.emplace_back(children->visits);
      search_debug[children->id].reward_debug.emplace_back(children->reward);
    }
  }
}
}  // namespace

void MCTS::Search(const std::vector<double> &leader_actions, const double ucb_c,
                  const double time_limit, const double time_step,
                  const MCTSNodePtr &root,
                  std::vector<MCTSNodeConstPtr> &best_path,
                  MCTSDebugInfo &debug_info) {
  if (nullptr == root ||
      ad_e2e::planning::math::Double::Compare(time_step, 0.0) ==
          ad_e2e::planning::math::Double::CompareType::EQUAL) {
    return;
  }
  best_path.clear();
  node_id_ = 0;
  auto start_time = std::chrono::high_resolution_clock::now();
  double elapsed_time = 0.0;
  std::map<int, SearchDebug> search_debug;
  std::set<int> leaf_debug;
  while (elapsed_time < time_limit) {
    MCTSNodePtr selected = Select(root, ucb_c);
    if (nullptr == selected) {
      break;
    }
    const int select_layer = std::floor(selected->current_time / time_step);
    if (selected != root && 0 == selected->visits) {
      UpdateChildNode(selected->GetParent(), selected);
    }
    if (!Expand(leader_actions, selected)) {
      break;
    }
    MCTSNodePtr leaf_node = Simulate(leader_actions, select_layer, selected);
    if (nullptr == leaf_node) {
      break;
    }
    if (!leaf_node->is_leaf) {
    }
    Backpropagate(leaf_node);
    auto cur_time = std::chrono::high_resolution_clock::now();
    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                       cur_time - start_time)
                       .count();
    if (enable_debug_) {
      BuildLeafDebug(*leaf_node, debug_info.leaf_debug);
      BuildSearchDebug(*root, debug_info.search_debug);
    }
  }

  MCTSNodeConstPtr current = root;
  while (nullptr != current) {
    best_path.push_back(current);
    current = FindBestChild(current);
    if (nullptr == current) {
      break;
    }
  }
}

MCTSNodePtr MCTS::Select(const MCTSNodePtr &root, const double ucb_c) {
  if (nullptr == root) {
    return nullptr;
  }
  MCTSNodePtr current = root;
  int total_visits = root->visits;
  int layer_idx = 0;
  while (!current->children.empty()) {
    if (layer_idx >= MAX_LAYER) {
      break;
    }
    double max_ucb = std::numeric_limits<double>::lowest();
    MCTSNodePtr best_child = nullptr;
    for (MCTSNodePtr child : current->children) {
      if (nullptr == child) {
        continue;
      }
      const double child_ucb = UCB(child, total_visits, ucb_c);
      if (child_ucb > max_ucb) {
        max_ucb = child_ucb;
        best_child = child;
      }
    }
    current = best_child;
    if (nullptr == current) {
      break;
    }
    total_visits = current->visits;
    layer_idx++;
  }
  return current;
}

bool MCTS::Expand(const std::vector<double> &leader_actions,
                  const MCTSNodePtr &node) {
  if (leader_actions.empty() || nullptr == node) {
    return false;
  }
  if (node->is_leaf || !node->children.empty()) {
    return true;
  }

  for (size_t i = 0; i < leader_actions.size(); i++) {
    if (PrePruning(node, leader_actions[i])) {
      continue;
    }
    MCTSNodePtr child_node = nullptr;
    CreateChildNode(node, leader_actions[i], child_node);
    if (nullptr == child_node) {
      continue;
    }
    node_id_++;
    child_node->id = node_id_;

    node->children.push_back(child_node);
  }

  if (!node->is_leaf && node->children.empty()) {
    return false;
  }
  return true;
}

MCTSNodePtr MCTS::Simulate(const std::vector<double> &leader_actions,
                           const int depth, const MCTSNodePtr &node) {
  if (nullptr == node || node->children.empty()) {
    return node;
  }

  if (depth >= MAX_LAYER) {
    return node;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, node->children.size() - 1);

  MCTSNodePtr child_node = node->children[dis(gen)];

  if (nullptr == child_node) {
    return node;
  }
  if (0 == child_node->visits) {
    UpdateChildNode(child_node->GetParent(), child_node);
  }
  if (child_node->is_leaf) {
    return child_node;
  }

  if (!Expand(leader_actions, child_node)) {
    return child_node;
  }
  return Simulate(leader_actions, depth + 1, child_node);
}

void MCTS::Backpropagate(MCTSNodePtr node) {
  int layer_idx = 0;
  while (node != nullptr) {
    if (layer_idx >= MAX_LAYER) {
      break;
    }
    if (node->visits == 0) {
      node->reward = 0.0;
    }
    node->visits++;
    if (node->visits < 2) {
      UpdateNodeStepReward(node);
    }

    double total_child_reward = 0.0;
    int explored_child_count = 0;
    for (MCTSNodePtr child : node->children) {
      if (child->visits > 0) {
        total_child_reward += child->reward;
        explored_child_count++;
      }
    }

    if (explored_child_count > 0) {
      const double average_child_reward =
          total_child_reward / explored_child_count * 0.9;
      node->reward = (average_child_reward + node->step_reward - node->reward) /
                         node->visits +
                     node->reward;
    } else {
      if (node->visits > 1) {
        node->reward =
            (node->step_reward - node->reward) / node->visits + node->reward;
      } else {
        node->reward = node->step_reward;
      }
    }
    node = node->GetMutableParent();
    layer_idx++;
  }
}

MCTSNodeConstPtr MCTS::FindBestChild(const MCTSNodeConstPtr &node) {
  if (node->children.empty()) {
    return nullptr;
  }
  double max_reward = std::numeric_limits<double>::lowest();
  MCTSNodeConstPtr best_child = nullptr;
  for (MCTSNodeConstPtr child : node->children) {
    if (nullptr == child) {
      continue;
    }
    if (child->reward > max_reward) {
      max_reward = child->reward;
      best_child = child;
    }
  }
  return best_child;
}

double MCTS::UCB(const MCTSNodePtr &node, double total_visits,
                 double c = 1.41) {
  if (node->visits <= 0) {
    return std::numeric_limits<double>::max();
  }
  return (node->reward / node->visits) +
         c * std::sqrt(std::log(total_visits) / node->visits);
}
}  // namespace planning
}  // namespace e2e_noa
