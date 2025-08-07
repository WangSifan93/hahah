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

class ProgressEvaluator {
 public:
  ProgressEvaluator() = default;

  ProgressEvaluator(EvaluationDeciderConfig::Progress config);

  /**
   * @brief evaluate trajectory progress cost
   *
   * @param proposal
   * @return Costs
   */
  Costs Evaluate(const Proposal& proposal);

  /**
   * @brief calculate max distance of proposals
   *
   * @param proposals
   * @return bool
   */
  bool CalculateMaxDistance(const std::vector<Proposal>& proposals);

 private:
  EvaluationDeciderConfig::Progress config_;
  LookupTable progress_cost_table_;
  double max_distance_ = 0.0;
};

}  // namespace planning
}  // namespace zark
