/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/common/lookup_table.h"
#include "apps/planning/src/common/proposal.h"

namespace zark {
namespace planning {

class SafetyEvaluator {
 public:
  SafetyEvaluator() = default;

  SafetyEvaluator(EvaluationDeciderConfig::Safety config);
  /**
   * @brief evaluate trajectory safety cost
   *
   * @param proposal
   * @param obstacles
   * @return Costs
   */
  Costs Evaluate(const Proposal& proposal,
                 const std::vector<const Obstacle*>& obstacles);

  /**
   * @brief evaluate trajectory slack cost
   *
   * @param proposal
   * @param costs
   */
  void EvaluateSlackCost(const Proposal& proposal, Costs& costs);

  /**
   * @brief evaluate trajectory lon/lat cost
   * rule: max{min(s1, l1), min(s2, l2), .... min(sn, ln)}
   *
   * @param proposal
   * @param obstacles
   * @param costs
   */
  void EvaluateLonLatCollisionCost(
      const Proposal& proposal, const std::vector<const Obstacle*>& obstacles,
      Costs& costs);

  void EvaluateLCDangerousCost(const Proposal& proposal,
                               const std::vector<const Obstacle*>& obstacles,
                               Costs& costs);

 private:
  /**
   * @brief evaluate lon/lat collision cost between obstacle and trajectory
   * point
   *
   * @param[in]  trajectory
   * @param[in]  trajectory_point
   * @param[in]  adc_box
   * @param[in]  obstacle
   * @param[in]  vehicle parameter
   * @param[out] max_collision_cost
   * @param[out] max_cost_sl
   * @param[out] max_cost_time
   */
  void EvaluateCollisionCostObstacle(
      const CorridorInfo::Type type, const DiscretizedTrajectory& trajectory,
      const ::common::TrajectoryPoint& trajectory_point,
      const ::math::Box2d& adc_box, const Obstacle& obstacle,
      const common::VehicleParam& param, double& max_collision_cost,
      std::pair<double, double>& max_cost_sl, double& max_cost_time,
      std::string& max_cost_obs_id);

 private:
  /**
   * @brief
   *
   * @param[in]  slack_slice
   * @param[in]  weight
   * @param[in]  time_weight_table
   * @param[in]  name
   * @param[out] costs
   */
  void SlackSliceCost(const Eigen::MatrixXd& slack_slice,
                      const std::vector<double>& weight,
                      const LookupTable& time_weight_table,
                      const std::vector<std::string>& name, Costs& costs);

  //
  SLBoundary GetObstacleBoxSL(const ::math::Box2d& obs,
                              const DiscretizedTrajectory& trajectory);
  //
  std::pair<double, double> GetSLDistanceOfObstacle(const SLBoundary& sl_obs,
                                                    const double s,
                                                    const double l,
                                                    const double length,
                                                    const double width);

 private:
  /**
   * @brief
   *
   * @param[in]  slack
   * @param[in]  slack_weight
   * @param[in]  time_weight_table
   * @param[in]  name
   * @param[out] costs
   */
  void SlackCost(const MPCData::Slacks::Slack& slack,
                 const EvaluationDeciderConfig::SlackWeight& slack_weight,
                 const LookupTable& time_weight_table,
                 const MPCData::Names& names, Costs& costs);

 private:
  EvaluationDeciderConfig::Safety config_;
  LookupTable safety_cost_table_;
  LookupTable collision_time_weight_table_;
  // assume the ego size: [5m, 2m], lane width: 3m
  LookupTable safety_lon_cost_table_;
  LookupTable safety_lat_cost_table_;
  LookupTable lc_time_table_;
};

}  // namespace planning
}  // namespace zark
