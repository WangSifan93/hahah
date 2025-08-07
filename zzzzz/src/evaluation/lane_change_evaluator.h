/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/common/proposal.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

class LaneChangeEvaluator {
 public:
  LaneChangeEvaluator() = delete;

  LaneChangeEvaluator(EvaluationDeciderConfig::LaneChange config);
  /**
   * @brief evaluate trajectory lane change cost
   *
   * @param proposal
   * @param last_corridor_type
   * @param t_staging
   * @return Costs
   */
  Costs Evaluate(const Proposal& proposal,
                 const CorridorInfo::Type last_corridor_type,
                 const double t_staging);

 private:
  /**
   * @brief evaluate lane change urgency cost
   *
   * @param mission
   * @param lon_mpc_data
   * @param proposal_corridor_type
   * @param t_staging
   * @param costs
   */
  void EvaluateLaneChangeUrgencyCost(
      const Mission& mission, const LonMPCData& lon_mpc_data,
      const CorridorInfo::Type proposal_corridor_type, const double t_staging,
      Costs& costs);

  /**
   * @brief evaluate lane change abort cost
   *
   * @param proposal_corridor_type
   * @param last_corridor_type
   * @param costs
   */
  void EvaluateLaneChangeAbortCost(
      const CorridorInfo::Type proposal_corridor_type,
      const CorridorInfo::Type last_corridor_type, Costs& costs);

  /**
   * @brief evaluate lane change state cost
   *
   * @param corridor_type
   * @param mission
   * @param costs
   */
  void EvaluateLaneChangeStateCost(const CorridorInfo::Type corridor_type,
                                   const Mission& mission, Costs& costs);

 private:
  EvaluationDeciderConfig::LaneChange config_;

 private:
  FRIEND_TEST(LaneChangeEvaluatorTest, TestEvaluateLaneChangeUrgencyCost);
  FRIEND_TEST(LaneChangeEvaluatorTest, TestEvaluateLaneChangeAbortCost);
};

}  // namespace planning
}  // namespace zark
