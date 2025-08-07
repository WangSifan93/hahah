/******************************************************************************
 * Copyright 2024 The zpilot . All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>

#include "apps/planning/src/evaluation/comfort_evaluator.h"
#include "apps/planning/src/evaluation/progress_evaluator.h"
#include "apps/planning/src/evaluation/safety_evaluator.h"
#include "apps/planning/src/evaluation/similarity_evaluator.h"
#include "apps/planning/src/evaluation/lane_change_evaluator.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/tasks/deciders/decider.h"

namespace zark {
namespace planning {

class EvaluationDecider : public Decider {
 public:
  EvaluationDecider(const TaskConfig& config,
                    const std::shared_ptr<DependencyInjector>& injector);
  /**
   * @brief
   *
   * @param frame
   * @return zark::::common::Status
   */
  ::common::Status Execute(Frame* frame);

 private:
  GapChoiceInfo ComputeLaneChangeGap(const Mission& mission,
                                     std::vector<Proposal>& proposals);

 private:
  SafetyEvaluator safety_evaluator_;
  ComfortEvaluator comfort_evaluator_;
  ProgressEvaluator progress_evaluator_;
  SimilarityEvaluator similarity_evaluator_;
  LaneChangeEvaluator lane_change_evaluator_;
};

}  // namespace planning
}  // namespace zark
