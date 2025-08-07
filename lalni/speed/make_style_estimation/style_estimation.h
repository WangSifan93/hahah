#ifndef PLANNER_SPEED_STYLE_ESTIMATION_H_
#define PLANNER_SPEED_STYLE_ESTIMATION_H_
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "../mcts_game_process/mcts_data_type.h"
#include "math/double.h"
#include "planner_params.pb.h"

namespace e2e_noa {
namespace planning {
class StyleEstimator {
 public:
  explicit StyleEstimator(
      SpeedPlanningParamsProto::StyleEstimationParams params,
      const size_t window_size, const double interact_position_x,
      double interact_position_y)
      : params_(params),
        N_(window_size),
        interact_position_x_(interact_position_x),
        interact_position_y_(interact_position_y),
        gamma_candidates_{0.2, 0.4, 0.6, 0.8},
        initial_weights_(gamma_candidates_.size(), 1.0){};

  double EstimateStyle(const std::vector<AgentStatus> &ego_vehicle_traj,
                       const std::vector<AgentStatus> &opponent_object_traj);

 private:
  double ComputeCumulativeReward(
      const std::vector<AgentStatus> &ego_vehicle_traj,
      const std::vector<AgentStatus> &opponent_object_traj, double gamma) const;

  double ComputeEgoReward(const AgentStatus &ego_vehicle,
                          const AgentStatus &opponent_object,
                          const double dis_ego, const double dis_opponent,
                          const double t_ego_arrive,
                          const double t_opponent_arrive) const;

  double ComputeOpponentReward(const AgentStatus &ego_vehicle,
                               const AgentStatus &opponent_object,
                               const double dis_ego, const double dis_opponent,
                               const double t_ego_arrive,
                               const double t_opponent_arrive) const;

  double CalculateSafetyReward(const AgentStatus &ego_vehicle,
                               const AgentStatus &opponent_object,
                               const double dis_ego, const double dis_opponent,
                               const double t_ego_arrive,
                               const double t_opponent_arrive) const;

  double CalculateEfficiencyReward(const double v, const double dis,
                                   const double t_arrive) const;

  double CalculateAccelerationReward(const double acc) const;

  std::vector<double> ApplySoftmaxWithInitialWeights(
      const std::vector<double> &rewards,
      const std::vector<double> &init_weights) const;

 private:
  size_t N_;
  std::vector<double> gamma_candidates_;
  std::vector<double> initial_weights_;
  const SpeedPlanningParamsProto::StyleEstimationParams params_;
  double interact_position_x_;
  double interact_position_y_;
};
}  // namespace planning
}  // namespace e2e_noa
#endif
