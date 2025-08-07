#ifndef PLANNER_SPEED_MCTS_LONGITUDINAL_H_
#define PLANNER_SPEED_MCTS_LONGITUDINAL_H_
#include <gtest/gtest.h>

#include "mcts.h"
#include "vehicle_status_update_model/vehicle_simulation_model.h"

namespace e2e_noa {
namespace planning {

class MCTSLongitudinal : public MCTS {
 public:
  MCTSLongitudinal() = delete;
  MCTSLongitudinal(const SpeedPlanningParamsProto::MCTSLonParamsProto* params)
      : params_(params) {}

  ~MCTSLongitudinal() = default;

  void InitOriginalSpeed(const double leader_v, const double follower_v) {
    leader_cur_v_ = leader_v;
    follower_cur_v_ = follower_v;
  }
  void InitLeaderModel(const double min_v, const double max_v,
                       const double time_step, const double jerk);

  void InitFollowerModel(const double min_v, const double max_v,
                         const double time_step, const double jerk);

 private:
  bool PrePruning(const MCTSNodePtr& node, const double leader_action) override;

  void CreateChildNode(const MCTSNodePtr& parent, const double leader_action,
                       MCTSNodePtr& child) override;

  void UpdateChildNode(const MCTSNodeConstPtr& parent,
                       const MCTSNodePtr& child) override;

  void UpdateNodeStepReward(const MCTSNodePtr& node) override;

  double CalculateStepReward(const double reward_safe, const double reward_eff,
                             const double reward_acc, const double reward_jerk,
                             const double reward_act, const double reward_heu);

  double CalculateSafetyReward(const MCTSNodeConstPtr& node,
                               const double t_leader_arrive,
                               const double t_follower_arrive);

  double CalculateEfficiencyReward(const MCTSNodeConstPtr& node,
                                   const double t_arrive);

  double CalculateAccelerationReward(const MCTSNodeConstPtr& node);

  double CalculateJerkReward(const MCTSNodeConstPtr& node);

  double CalculateActionSmoothReward(const MCTSNodeConstPtr& node);

  double CalculateHeuristicsReward(const MCTSNodeConstPtr& node,
                                   const double t_leader_arrive,
                                   const double t_follower_arrive);
  void UpdateLeaderState(const MCTSNodeConstPtr& parent,
                         const double leader_action, NodeState& state);

  bool IsLeaf(const MCTSNodeConstPtr& node);

 private:
  double time_step_ = 0.5;
  double leader_max_v_ = 16.6;
  double leader_cur_v_ = 16.6;
  double follower_cur_v_ = 16.6;
  VehicleSimulationModel leader_model_;
  VehicleSimulationModel follower_model_;
  const SpeedPlanningParamsProto::MCTSLonParamsProto* params_ = nullptr;
};
}  // namespace planning
}  // namespace e2e_noa
#endif
