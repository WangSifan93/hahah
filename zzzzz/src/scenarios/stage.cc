/******************************************************************************
 * Copyright 2023 The zpilot Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#include "stage.h"

#include <chrono>
#include <unordered_map>
#include <utility>

#include "apps/planning/src/common/clock.h"
#include "apps/planning/src/common/trajectory/publishable_trajectory.h"
#include "apps/planning/src/tasks/task_factory.h"

namespace zark {
namespace planning {
namespace scenario {

// using zark::cyber::Clock;
using namespace zark::common;

namespace {
// constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
// constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

Stage::Stage(const ScenarioConfig::StageConfig& config,
             const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), injector_(injector) {
  name_ = StageType_Name(config_.stage_type());
  next_stage_ = config_.stage_type();
  std::unordered_map<TaskConfig::TaskType, const TaskConfig*, std::hash<int>>
      config_map;
  for (const auto& task_config : config_.task_config()) {
    config_map[task_config.task_type()] = &task_config;
  }
  for (std::size_t i = 0; i < config_.task_type().size(); ++i) {
    auto task_type = config_.task_type().at(i);
    ACHECK(config_map.find(task_type) != config_map.end())
        << "Task: " << (task_type) << " used but not configured";
    auto iter = tasks_.find(task_type);
    if (iter == tasks_.end()) {
      auto ptr = TaskFactory::CreateTask(*config_map[task_type], injector_);
      task_list_.push_back(ptr.get());
      tasks_[task_type] = std::move(ptr);
    } else {
      task_list_.push_back(iter->second.get());
    }
  }
}

const std::string& Stage::Name() const { return name_; }

Task* Stage::FindTask(TaskConfig::TaskType task_type) const {
  auto iter = tasks_.find(task_type);
  if (iter == tasks_.end()) {
    return nullptr;
  } else {
    return iter->second.get();
  }
}

Stage::StageStatus Stage::FinishScenario() {
  next_stage_ = StageType::NO_STAGE;
  return Stage::FINISHED;
}

std::string Stage::StageType_Name(StageType stage_type) {
  std::string res = "NO_STAGE";

  switch (stage_type) {
    case LANE_FOLLOW_V2_DEFAULT_STAGE:
      res = "LANE_FOLLOW_V2_DEFAULT_STAGE";
      break;
    default:
      break;
  }
  return res;
}

}  // namespace scenario
}  // namespace planning
}  // namespace zark
