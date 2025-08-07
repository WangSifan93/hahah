/******************************************************************************
 * Copyright 2023 The zpilot Author Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <unordered_map>

#include "status.h"
#include "apps/planning/src/common/macros.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/scenarios/scenario.h"

namespace zark {
namespace planning {
namespace scenario {

class ScenarioManager final {
 public:
  ScenarioManager() = delete;

  explicit ScenarioManager(const std::shared_ptr<DependencyInjector>& injector);

  bool Init(const PlanningConfig& planning_config);

  Scenario* mutable_scenario() { return current_scenario_.get(); }

  DependencyInjector* injector() { return injector_.get(); }

  void Update(const ::common::TrajectoryPoint& ego_point, const Frame& frame);

 private:
  std::unique_ptr<Scenario> CreateScenario(ScenarioType scenario_type);

  void RegisterScenarios();

  void ScenarioDispatch(const Frame& frame);

 private:
  std::shared_ptr<DependencyInjector> injector_;
  PlanningConfig planning_config_;
  std::unordered_map<ScenarioType, ScenarioConfig, std::hash<int>> config_map_;
  std::unique_ptr<Scenario> current_scenario_;
  ScenarioType default_scenario_type_;
  ScenarioContext scenario_context_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace zark
