/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <iostream>

#include "messages/mapfusion/routing.pb.h"
#include "apps/planning/src/common/vehicle_config.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/planning_msgs/local_route_config.h"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "conf_gflags.h"
#include "nlohmann/json.hpp"

namespace zark {
namespace planning {

class Config {
 public:
  Config() = default;

  Config(const std::string &config_file);

  void SetScenarioConfig(ScenarioConfig &scenario_config,
                         PlanningConfig &planning_config);

  void SetPlanningConfig(PlanningConfig &planning_config);
  void SetDefaultTaskConfig(PlanningConfig &planning_config);

  void SetVehicleConfig(common::VehicleConfig &config);

  void SwitchTaskConfig(TaskConfig &task_config, const nlohmann::json jnode,
                        const TaskConfig::TaskType task_type);

  void SetLocalRouteConfig(LocalRouteConfig &refline_config);

  /**
   * @brief get lookup table config from the table jnode
   * @param lookup_table_config
   * @param table_jnode
   * @return the LookupTableConfig
   */
  LookupTableConfig GetLookupTableConfig(const nlohmann::json &table_jnode);

  /**
   * @brief Set the Traffic Rule Decider Config object
   *
   * @param jnode
   * @return TrafficRuleDeciderConfig
   */
  TrafficRuleDeciderConfig SetTrafficRuleDeciderConfig(
      const nlohmann::json &jnode);

  /**
   * @brief Set the Mission Decider Config object
   *
   * @param jnode
   * @return MissionDeciderConfig
   */
  MissionDeciderConfig SetMissionDeciderConfig(const nlohmann::json &jnode);

  /**
   * @brief Set the Longitudinal Optimizer Config object
   *
   * @param jnode json node of longitudinal optimizer config
   * @return the LongitudinalOptimizerConfig
   */
  LongitudinalOptimizerConfig SetLongitudinalOptimizerConfig(
      const nlohmann::json &jnode);

  /**
   * @brief Set the Lateral Motion Param Config object
   */
  LateralOptimizerConfig SetLateralOptimizerConfig(const nlohmann::json &jnode);

  /**
   * @brief Set the Evaluation Decider Config object
   *
   * @param jnode json node of evaluation optimizer config
   * @return EvaluationDeciderConfig
   */
  EvaluationDeciderConfig SetEvaluationDeciderConfig(
      const nlohmann::json &jnode);

 private:
  nlohmann::json jnode_;
  // TaskConfig
  TaskConfig::task_config_union task_config_;
};

}  // namespace planning
}  // namespace zark
