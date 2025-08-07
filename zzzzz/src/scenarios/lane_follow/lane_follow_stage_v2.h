/******************************************************************************
 * Copyright 2023 The zpilot . All Rights Reserved.
 *****************************************************************************/
/**
 * @lane_follow_stage_v2
 **/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "pnc_point.h"
#include "status.h"
#include "factory.h"
#include "apps/planning/src/common/local_route/local_route.h"
#include "apps/planning/src/common/local_route/local_route_point.h"
#include "apps/planning/src/planning_msgs/planning.h"
#include "apps/planning/src/scenarios/scenario.h"
#include "apps/planning/src/scenarios/stage.h"
#include "apps/planning/src/tasks/task.h"

namespace zark {
namespace planning {
namespace scenario {
namespace lane_follow {

class LaneFollowStageV2 : public Stage {
 public:
  LaneFollowStageV2(const ScenarioConfig::StageConfig& config,
                    const std::shared_ptr<DependencyInjector>& injector);

  StageStatus Process(const ::common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

  ::common::Status PlanOnLocalRoute(
      const ::common::TrajectoryPoint& planning_start_point, Frame* frame);

 private:
  ScenarioConfig config_;
  std::unique_ptr<Stage> stage_;
};

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace zark
