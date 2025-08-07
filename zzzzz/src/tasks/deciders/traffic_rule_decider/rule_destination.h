/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file rule_destination.h
 **/

#pragma once

#include "apps/planning/src/planning_msgs/task_config.h"
#include "status.h"
#include "apps/planning/src/common/frame.h"

namespace zark {
namespace planning {
using RuleDestinationConfig = TrafficRuleDeciderConfig::Destination;
using Status = ::common::Status;

class RuleDestination {
 public:
  RuleDestination(const RuleDestinationConfig& config) : config_(config){};

  virtual ~RuleDestination() = default;

  Status ApplyRule(Frame& frame);

 private:
  RuleDestinationConfig config_;
};

}  // namespace planning
}  // namespace zark
