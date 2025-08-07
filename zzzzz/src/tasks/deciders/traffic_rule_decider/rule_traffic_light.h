/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file rule_traffic_light.h
 **/

#pragma once

#include "apps/planning/src/planning_msgs/task_config.h"
#include "status.h"
#include "apps/planning/src/common/frame.h"

namespace zark {
namespace planning {
using TrafficLightConfig = TrafficRuleDeciderConfig::TrafficLight;

class RuleTrafficLight {
 public:
  RuleTrafficLight(const TrafficLightConfig& config) : config_(config){};

  virtual ~RuleTrafficLight() = default;

  ::common::Status ApplyRule(Frame* const frame) { return Status::OK(); };

 private:
  void MakeDecisions(Frame* const frame);

  TrafficLightConfig config_;
};

}  // namespace planning
}  // namespace zark
