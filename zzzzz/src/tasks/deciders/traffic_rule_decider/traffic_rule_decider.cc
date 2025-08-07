/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file traffic_rule_decider.cc
 **/

#include "apps/planning/src/tasks/deciders/traffic_rule_decider/traffic_rule_decider.h"

namespace zark {
namespace planning {

TrafficRuleDecider::TrafficRuleDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Task(config, injector),
      config_(config.task_config().traffic_rule_decider_config) {
  rule_traffic_light_ =
      std::make_unique<RuleTrafficLight>(config_.traffic_light);
  rule_destination_ = std::make_unique<RuleDestination>(config_.destination);
  rule_crosswalk_ = std::make_unique<RuleCrosswalk>(config_.crosswalk);
}

Status TrafficRuleDecider::Execute(Frame* frame) {
  CHECK_NOTNULL(frame);

  Status status;
  // status = rule_traffic_light_->ApplyRule(frame);
  // if (!status.ok()) return status;

  status = rule_destination_->ApplyRule(*frame);
  if (!status.ok()) return status;

  // status = rule_crosswalk_->ApplyRule(frame);
  // if (!status.ok()) return status;

  return Status::OK();
}

}  // namespace planning
}  // namespace zark
