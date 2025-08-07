
#ifndef PLANNER_SPEED_MCTS_H_
#define PLANNER_SPEED_MCTS_H_
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "math/double.h"
#include "mcts_data_type.h"

namespace e2e_noa {
namespace planning {

class MCTS {
 public:
  MCTS() = default;

  virtual ~MCTS() = default;

  void Search(const std::vector<double>& leader_actions, const double ucb_c,
              const double time_limit, const double time_step,
              const MCTSNodePtr& root, std::vector<MCTSNodeConstPtr>& best_path,
              MCTSDebugInfo& debug_info);

 private:
  MCTSNodePtr Select(const MCTSNodePtr& root, const double ucb_c);

  bool Expand(const std::vector<double>& leader_actions,
              const MCTSNodePtr& node);

  MCTSNodePtr Simulate(const std::vector<double>& leader_actions,
                       const int depth, const MCTSNodePtr& node);

  void Backpropagate(MCTSNodePtr node);

  MCTSNodeConstPtr FindBestChild(const MCTSNodeConstPtr& node);

  double UCB(const MCTSNodePtr& node, const double total_visits,
             const double c);

  virtual bool PrePruning(const MCTSNodePtr& node,
                          const double leader_action) = 0;

  virtual void CreateChildNode(const MCTSNodePtr& parent,
                               const double leader_action,
                               MCTSNodePtr& child) = 0;

  virtual void UpdateChildNode(const MCTSNodeConstPtr& parent,
                               const MCTSNodePtr& child) = 0;

  virtual void UpdateNodeStepReward(const MCTSNodePtr& node) = 0;

 protected:
  bool enable_debug_ = true;
  bool enable_lon_debug_ = true;
  bool enable_spt_debug_ = true;

 private:
  int node_id_ = 0;
};
}  // namespace planning
}  // namespace e2e_noa
#endif
