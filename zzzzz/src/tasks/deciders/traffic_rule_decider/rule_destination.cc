/******************************************************************************
 * Copyright 2024 The zark. All Rights Reserved.
 *****************************************************************************/

/**
 * @file rule_destination.cc
 **/

#include "apps/planning/src/tasks/deciders/traffic_rule_decider/rule_destination.h"

namespace zark {
namespace planning {
using Status = ::common::Status;

Status RuleDestination::ApplyRule(Frame& frame) {
  if (!config_.enable) {
    return Status::OK();
  }
  // TODO:
  // 1. get destination info (map service is to provide this info)
  // 2. convert destination info to destination_s
  double destination_s = 1.0e3;
  if (destination_s > 0) {
    for (auto& local_route : frame.MutableLocalRoutes()) {
      local_route.SetDestinationS(destination_s);
    }
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace zark
