#include "style_estimation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#include "math/double.h"
namespace e2e_noa {
namespace planning {
constexpr double kEpsilon = 1.0e-6;
double StyleEstimator::EstimateStyle(
    const std::vector<AgentStatus> &ego_vehicle_traj,
    const std::vector<AgentStatus> &opponent_object_traj) {
  std::vector<double> rewards;
  for (const auto &gamma : gamma_candidates_) {
    double reward =
        ComputeCumulativeReward(ego_vehicle_traj, opponent_object_traj, gamma);
    rewards.push_back(reward);
  }

  std::vector<double> weights =
      ApplySoftmaxWithInitialWeights(rewards, initial_weights_);

  // Weighted average to estimate pedestrian style gamma
  double estimated_gamma = 0.0;
  for (size_t i = 0; i < gamma_candidates_.size(); ++i) {
    estimated_gamma += weights[i] * gamma_candidates_[i];
  }

  return estimated_gamma;
}

double StyleEstimator::ComputeCumulativeReward(
    const std::vector<AgentStatus> &ego_vehicle_traj,
    const std::vector<AgentStatus> &opponent_object_traj, double gamma) const {
  double total_reward = 0.0;
  for (size_t t = 0; t < N_; ++t) {
    const double dis_ego =
        std::sqrt(std::pow(ego_vehicle_traj[t].x - interact_position_x_, 2) +
                  std::pow(ego_vehicle_traj[t].y - interact_position_y_, 2));
    const double dis_opponent = std::sqrt(
        std::pow(opponent_object_traj[t].x - interact_position_x_, 2) +
        std::pow(opponent_object_traj[t].y - interact_position_y_, 2));
    const double t_ego_arrive =
        dis_ego / std::max(kEpsilon, ego_vehicle_traj[t].v + 0.01);
    const double t_opponent_arrive =
        dis_opponent / std::max(kEpsilon, opponent_object_traj[t].v);

    double r =
        (1 - gamma) * ComputeEgoReward(
                          ego_vehicle_traj[t], opponent_object_traj[t], dis_ego,
                          dis_opponent, t_ego_arrive, t_opponent_arrive) +
        gamma * ComputeOpponentReward(
                    ego_vehicle_traj[t], opponent_object_traj[t], dis_ego,
                    dis_opponent, t_ego_arrive, t_opponent_arrive);
    total_reward += r;
  }
  return total_reward;
}

double StyleEstimator::ComputeEgoReward(const AgentStatus &ego_vehicle,
                                        const AgentStatus &opponent_object,
                                        const double dis_ego,
                                        const double dis_opponent,
                                        const double t_ego_arrive,
                                        const double t_opponent_arrive) const {
  const double reward_safe =
      CalculateSafetyReward(ego_vehicle, opponent_object, dis_ego, dis_opponent,
                            t_ego_arrive, t_opponent_arrive);
  const double reward_eff =
      CalculateEfficiencyReward(ego_vehicle.v, dis_ego, t_ego_arrive);
  const double reward_acc = CalculateAccelerationReward(ego_vehicle.a);

  double reward = params_.w_acc() * reward_acc + params_.w_eff() * reward_eff +
                  params_.w_safe() * reward_safe;
  return reward;
}

double StyleEstimator::ComputeOpponentReward(
    const AgentStatus &ego_vehicle, const AgentStatus &opponent_object,
    const double dis_ego, const double dis_opponent, const double t_ego_arrive,
    const double t_opponent_arrive) const {
  const double reward_safe =
      CalculateSafetyReward(ego_vehicle, opponent_object, dis_ego, dis_opponent,
                            t_ego_arrive, t_opponent_arrive);
  const double reward_eff = CalculateEfficiencyReward(
      opponent_object.v, dis_opponent, t_opponent_arrive);
  const double reward_acc = CalculateAccelerationReward(opponent_object.a);

  double reward = params_.w_acc() * reward_acc + params_.w_eff() * reward_eff +
                  params_.w_safe() * reward_safe;
  return reward;
}

std::vector<double> StyleEstimator::ApplySoftmaxWithInitialWeights(
    const std::vector<double> &rewards,
    const std::vector<double> &init_weights) const {
  std::vector<double> weighted_exp;
  double sum = 0.0;

  for (size_t i = 0; i < rewards.size(); ++i) {
    double w = std::exp(rewards[i]) * init_weights[i];
    weighted_exp.push_back(w);
    sum += w;
  }

  // normalization
  if (std::abs(sum) < kEpsilon) {
    if (sum < 0) {
      sum = -kEpsilon;
    } else {
      sum = kEpsilon;
    }
  }
  std::vector<double> result;
  for (double val : weighted_exp) {
    result.push_back(val / sum);
  }

  return result;
}

double StyleEstimator::CalculateSafetyReward(
    const AgentStatus &ego_vehicle, const AgentStatus &opponent_object,
    const double dis_ego, const double dis_opponent, const double t_ego_arrive,
    const double t_opponent_arrive) const {
  constexpr double kVThreshold = 1.0e-1;

  const double t_diff = t_ego_arrive - t_opponent_arrive;  // 到达时间差时间计算
  const double v_ego = ego_vehicle.v;
  const double v_opponent = opponent_object.v;

  if (ad_e2e::planning::math::Double::Compare(dis_ego,
                                              params_.threshold_collision()) ==
          ad_e2e::planning::math::Double::CompareType::LESS &&
      ad_e2e::planning::math::Double::Compare(dis_opponent,
                                              params_.threshold_collision()) ==
          ad_e2e::planning::math::Double::CompareType::
              LESS) {  // Priority 1: Collision imminent
    return params_.reward_min();
  }

  if (ad_e2e::planning::math::Double::Compare(std::abs(v_ego - v_opponent),
                                              kVThreshold) ==
      ad_e2e::planning::math::Double::CompareType::LESS) {  // Priority 2: Speed
                                                            // matching
    return params_.reward_max();
  }

  if (ad_e2e::planning::math::Double::Compare(std::abs(t_diff),
                                              params_.t_diff_max()) ==
      ad_e2e::planning::math::Double::CompareType::LESS) {  // Priority 3: Time
                                                            // to collision
    return (std::abs(t_diff) / std::max(kEpsilon, params_.t_diff_max())) *
           params_.reward_max();
  } else {
    return params_.reward_max();
  }
  return 0.0;
}

double StyleEstimator::CalculateEfficiencyReward(const double v,
                                                 const double dis,
                                                 const double t_arrive) const {
  const double t_expected =
      dis / std::max(v * params_.radio_v_perf(), params_.v_perf());
  if (ad_e2e::planning::math::Double::Compare(t_arrive, t_expected) ==
          ad_e2e::planning::math::Double::CompareType::LESS ||  // means faster
                                                                // than expected
      ad_e2e::planning::math::Double::Compare(t_arrive, 0) ==
          ad_e2e::planning::math::Double::CompareType::EQUAL) {  // means had
                                                                 // arrived
    return params_.reward_max();
  }
  return (t_expected / std::max(kEpsilon, t_arrive)) * params_.reward_max();
};

double StyleEstimator::CalculateAccelerationReward(const double acc) const {
  if (ad_e2e::planning::math::Double::Compare(acc, 0) !=
          ad_e2e::planning::math::Double::CompareType::LESS &&
      ad_e2e::planning::math::Double::Compare(acc, params_.a_max()) ==
          ad_e2e::planning::math::Double::CompareType::
              LESS) {  // Positive aeleration case
    return (1.0 - acc / std::max(kEpsilon, params_.a_max())) *
           params_.reward_max();
  } else if (ad_e2e::planning::math::Double::Compare(acc, 0) ==
                 ad_e2e::planning::math::Double::CompareType::LESS &&
             ad_e2e::planning::math::Double::Compare(acc, params_.a_min()) ==
                 ad_e2e::planning::math::Double::CompareType::
                     GREATER) {  // Negative aeleration
                                 // case
    const double abs_ratio =
        std::abs(acc / std::min(-kEpsilon, params_.a_min()));
    return (1.0 - abs_ratio) * params_.reward_max();
  }
  return 0.0;
};

}  // namespace planning
}  // namespace e2e_noa