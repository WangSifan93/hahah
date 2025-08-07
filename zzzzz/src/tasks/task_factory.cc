/******************************************************************************
 * Copyright 2023 The zpilot Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#include "apps/planning/src/tasks/task_factory.h"
#include "status.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/tasks/deciders/longitudinal_decider/longitudinal_decider.h"
#include "apps/planning/src/tasks/deciders/lateral_decider/lateral_decider.h"
#include "apps/planning/src/tasks/task.h"
#include "apps/planning/src/tasks/optimizers/lateral_optimizer/lateral_optimizer.h"
#include "apps/planning/src/tasks/deciders/evaluation_decider/evaluation_decider.h"
#include "apps/planning/src/tasks/deciders/mission_decider/mission_decider.h"
#include "apps/planning/src/tasks/optimizers/longitudinal_optimizer/longitudinal_optimizer.h"
#include "apps/planning/src/tasks/deciders/traffic_rule_decider/traffic_rule_decider.h"

namespace zark {
namespace planning {

::util::Factory<TaskConfig::TaskType, Task,
                Task* (*)(const TaskConfig& config,
                          const std::shared_ptr<DependencyInjector>& injector),
                std::unordered_map<TaskConfig::TaskType,
                                   Task* (*)(const TaskConfig& config,
                                             const std::shared_ptr<
                                                 DependencyInjector>& injector),
                                   std::hash<int>>>
    TaskFactory::task_factory_;

std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
    TaskFactory::default_task_configs_;

void TaskFactory::Init(const PlanningConfig& config,
                       const std::shared_ptr<DependencyInjector>& injector) {
  // deciders
  task_factory_.Register(
      TaskConfig::TRAFFIC_RULE_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new TrafficRuleDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::MISSION_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new MissionDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::LATERAL_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new LateralDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::LONGITUDINAL_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new LongitudinalDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::LATERAL_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new LateralOptimizer(config, injector);
      });
  task_factory_.Register(
      TaskConfig::LONGITUDINAL_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new LongitudinalOptimizer(config, injector);
      });
  task_factory_.Register(
      TaskConfig::EVALUATION_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new EvaluationDecider(config, injector);
      });

  for (const auto& default_task_config : config.default_task_config()) {
    default_task_configs_[default_task_config.task_type()] =
        default_task_config;
  }
}

std::unique_ptr<Task> TaskFactory::CreateTask(
    const TaskConfig& task_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  TaskConfig merged_config;
  if (default_task_configs_.find(task_config.task_type()) !=
      default_task_configs_.end()) {
    merged_config = default_task_configs_[task_config.task_type()];
  }
  merged_config.MergeFrom(task_config);
  return task_factory_.CreateObject(task_config.task_type(), merged_config,
                                    injector);
}

}  // namespace planning
}  // namespace zark
