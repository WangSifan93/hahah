/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/common/proposal.h"
#include "apps/planning/src/common/lookup_table.h"

namespace zark {
namespace planning {

class ComfortEvaluator {
 public:
  ComfortEvaluator() = default;

  ComfortEvaluator(EvaluationDeciderConfig::Comfort config);
  /**
   * @brief evaluate trajectory comfort cost
   *
   * @param proposal
   * @return Costs
   */
  Costs Evaluate(const Proposal& proposal);

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
  EvaluationDeciderConfig::Comfort config_;
  LookupTable time_weight_table_{};
};

}  // namespace planning
}  // namespace zark
