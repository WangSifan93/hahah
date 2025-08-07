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

class SimilarityEvaluator {
 public:
  SimilarityEvaluator() = default;

  SimilarityEvaluator(EvaluationDeciderConfig::Similarity config);

  /**
   * @brief evaluate trajectory similarity cost
   *
   * @param proposal
   * @param traj_prev
   * @return Costs
   */
  Costs Evaluate(const Proposal& proposal,
                 const DiscretizedTrajectory& traj_prev);

 private:
  EvaluationDeciderConfig::Similarity config_;
  LookupTable similarity_cost_table_;
};

}  // namespace planning
}  // namespace zark
