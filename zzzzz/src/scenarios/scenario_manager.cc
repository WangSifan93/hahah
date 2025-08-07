/******************************************************************************
 * Copyright 2023 The zpilot Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

#include <algorithm>
#include <string>
#include <vector>
#include "apps/planning/src/scenarios/scenario_manager.h"
#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "point_factory.h"
#include "apps/planning/src/common/vehicle_state/vehicle_state_provider.h"
#include "apps/planning/src/common/util/util.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/reference_line_provider/pnc_map/path.h"
#include "apps/planning/src/scenarios/lane_follow/lane_follow_scenario_v2.h"

namespace zark {
namespace planning {
namespace scenario {

using namespace hdmap;

ScenarioManager::ScenarioManager(
    const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

bool ScenarioManager::Init(const PlanningConfig& planning_config) {
  planning_config_ = (planning_config);
  RegisterScenarios();
  default_scenario_type_ = ScenarioType::LANE_FOLLOW_V2;
  current_scenario_ = CreateScenario(default_scenario_type_);
  return true;
}

std::unique_ptr<Scenario> ScenarioManager::CreateScenario(
    ScenarioType scenario_type) {
  std::unique_ptr<Scenario> ptr;

  switch (scenario_type) {
    case ScenarioType::LANE_FOLLOW_V2:
      ptr.reset(new scenario::lane_follow::LaneFollowScenarioV2(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    default:
      return nullptr;
  }

  if (ptr != nullptr) {
    ptr->Init();
  }
  return ptr;
}

void ScenarioManager::RegisterScenarios() {
  // lane_follow
  ACHECK(Scenario::LoadConfig(
      "apps/planning/config/json/lane_follow_v2_config.json",
      &config_map_[ScenarioType::LANE_FOLLOW_V2], planning_config_));
}

void ScenarioManager::Update(const ::common::TrajectoryPoint& ego_point,
                             const Frame& frame) {
  ScenarioDispatch(frame);
}

void ScenarioManager::ScenarioDispatch(const Frame& frame) {
  ACHECK(!frame.LocalRoutes().empty());

  ScenarioType scenario_type = ScenarioType::LANE_FOLLOW_V2;
  if (scenario_type == default_scenario_type_) {
    switch (current_scenario_->scenario_type()) {
      case ScenarioType::LANE_FOLLOW_V2:
        if (current_scenario_->GetStatus() !=
            Scenario::ScenarioStatus::STATUS_DONE) {
          scenario_type = current_scenario_->scenario_type();
        }
        break;
      default:
        break;
    }
  }

  if (current_scenario_->scenario_type() != scenario_type) {
    current_scenario_ = CreateScenario(scenario_type);
  }
}

}  // namespace scenario
}  // namespace planning
}  // namespace zark
