/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file rule_crosswalk.h
 **/

#pragma once

#include "apps/planning/src/planning_msgs/task_config.h"
#include "status.h"
#include "apps/planning/src/common/frame.h"

namespace zark {
namespace planning {
using CrosswalkConfig = TrafficRuleDeciderConfig::Crosswalk;

class RuleCrosswalk {
 public:
  RuleCrosswalk(const CrosswalkConfig& config) : config_(config){};

  virtual ~RuleCrosswalk() = default;

  ::common::Status ApplyRule(Frame* const frame) { return Status::OK(); };

 private:
  void MakeDecisions(Frame* const frame);

  CrosswalkConfig config_;
};

}  // namespace planning
}  // namespace zark
