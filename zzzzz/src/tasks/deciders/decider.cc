/******************************************************************************
 * Copyright 2023 The zpilot . All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#include "apps/planning/src/tasks/deciders/decider.h"

#include <memory>

namespace zark {
namespace planning {

Decider::Decider(const TaskConfig& config) : Task(config) {}

Decider::Decider(const TaskConfig& config,
                 const std::shared_ptr<DependencyInjector>& injector)
    : Task(config, injector) {}

::common::Status Decider::Execute(Frame* frame) {
  Task::Execute(frame);
  return Process(frame);
}

}  // namespace planning
}  // namespace zark
