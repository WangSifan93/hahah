/******************************************************************************
 * Copyright 2023 The zpilot Author Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#include "apps/planning/src/tasks/task.h"

#include <memory>

namespace zark {
namespace planning {

Task::Task(const TaskConfig& config) : config_(config) {
  // config_.set_task_config(config);
  name_ = TaskConfig::TaskType_Name(config_.task_type());
}

Task::Task(const TaskConfig& config,
           const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), injector_(injector) {
  name_ = TaskConfig::TaskType_Name(config_.task_type());
}

const std::string& Task::Name() const { return name_; }

Status Task::Execute(Frame* frame) {
  frame_ = frame;
  return Status::OK();
}

}  // namespace planning
}  // namespace zark
