/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file traffic_rule_decider.h
 **/

#pragma once

#include "apps/planning/src/tasks/task.h"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "apps/planning/src/tasks/deciders/traffic_rule_decider/rule_destination.h"
#include "apps/planning/src/tasks/deciders/traffic_rule_decider/rule_crosswalk.h"
#include "apps/planning/src/tasks/deciders/traffic_rule_decider/rule_traffic_light.h"

namespace zark {
namespace planning {

class TrafficRuleDecider : public Task {
 public:
  TrafficRuleDecider(const TaskConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector);

  virtual ~TrafficRuleDecider() = default;

  Status Execute(Frame* frame);

 private:
  TrafficRuleDeciderConfig config_;
  std::unique_ptr<RuleTrafficLight> rule_traffic_light_;
  std::unique_ptr<RuleCrosswalk> rule_crosswalk_;
  std::unique_ptr<RuleDestination> rule_destination_;
};

}  // namespace planning
}  // namespace zark
