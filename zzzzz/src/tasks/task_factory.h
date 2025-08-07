/******************************************************************************
 * Copyright 2023 The zpilot Author Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <memory>
#include <unordered_map>

#include "factory.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/tasks/task.h"

namespace zark {
namespace planning {

class TaskFactory {
 public:
  static void Init(const PlanningConfig &config,
                   const std::shared_ptr<DependencyInjector> &injector);
  static std::unique_ptr<Task> CreateTask(
      const TaskConfig &task_config,
      const std::shared_ptr<DependencyInjector> &injector);

 private:
  static ::util::Factory<
      TaskConfig::TaskType, Task,
      Task *(*)(const TaskConfig &config,
                const std::shared_ptr<DependencyInjector> &injector),
      std::unordered_map<
          TaskConfig::TaskType,
          Task *(*)(const TaskConfig &config,
                    const std::shared_ptr<DependencyInjector> &injector),
          std::hash<int>>>
      task_factory_;
  static std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
      default_task_configs_;
};

}  // namespace planning
}  // namespace zark
