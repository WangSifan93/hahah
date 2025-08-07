/******************************************************************************
 * Copyright 2024 The zpilot . All Rights Reserved.
 *****************************************************************************/
/**
 * @lane_follow_scenario_v2.h
 **/

#pragma once

#include <memory>

#include "apps/planning/src/scenarios/scenario.h"
#include "apps/planning/src/scenarios/stage.h"
#include "apps/planning/src/tasks/task.h"

namespace zark {
namespace planning {
namespace scenario {
namespace lane_follow {

class LaneFollowScenarioV2 : public Scenario {
 public:
  LaneFollowScenarioV2(const ScenarioConfig& config,
                       const ScenarioContext* context,
                       const std::shared_ptr<DependencyInjector>& injector)
      : Scenario(config, context, injector) {}

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config,
      const std::shared_ptr<DependencyInjector>& injector) override;
};

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace zark
