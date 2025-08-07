/******************************************************************************
 * Copyright 2024 The zpilot . All Rights Reserved.
 *****************************************************************************/
/**
 * @file
 **/

#include "apps/planning/src/scenarios/lane_follow/lane_follow_scenario_v2.h"

#include "apps/planning/src/common/log.h"
#include "apps/planning/src/scenarios/lane_follow/lane_follow_stage_v2.h"

namespace zark {
namespace planning {
namespace scenario {
namespace lane_follow {

std::unique_ptr<Stage> LaneFollowScenarioV2::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (stage_config.stage_type() != StageType::LANE_FOLLOW_V2_DEFAULT_STAGE) {
    AERROR << "Follow lane does not support stage type: "
           << Stage::StageType_Name(stage_config.stage_type());
    return nullptr;
  }
  return std::unique_ptr<Stage>(new LaneFollowStageV2(stage_config, injector));
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace zark
