/******************************************************************************
 * Copyright 2023 The zpilot Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "apps/planning/src/common/frame.h"
#include "status.h"
#include "factory.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/tasks/task.h"

namespace zark {
namespace planning {
namespace scenario {

class Stage {
 public:
  enum StageStatus {
    ERROR = 1,
    READY = 2,
    RUNNING = 3,
    FINISHED = 4,
  };

  Stage(const ScenarioConfig::StageConfig& config,
        const std::shared_ptr<DependencyInjector>& injector);

  virtual ~Stage() = default;

  const ScenarioConfig::StageConfig& config() const { return config_; }

  StageType stage_type() const { return config_.stage_type(); }

  /**
   * @brief Each stage does its business logic inside Process function.
   * If the stage want to transit to a different stage after finish,
   * it should set the type of 'next_stage_'.
   */
  virtual StageStatus Process(
      const ::common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;

  /**
   * @brief The sequence of tasks inside the stage. These tasks usually will be
   * executed in order.
   */
  const std::vector<Task*>& TaskList() const { return task_list_; }

  const std::string& Name() const;

  template <typename T>
  T* GetContextAs() {
    return static_cast<T*>(context_);
  }

  void SetContext(void* context) { context_ = context; }

  Task* FindTask(TaskConfig::TaskType task_type) const;

  StageType NextStage() const { return next_stage_; }

  static std::string StageType_Name(StageType stage_type);

 protected:
  virtual Stage::StageStatus FinishScenario();

 protected:
  std::map<TaskConfig::TaskType, std::unique_ptr<Task>> tasks_;
  std::vector<Task*> task_list_;
  ScenarioConfig::StageConfig config_;
  StageType next_stage_;
  void* context_ = nullptr;
  std::string name_;
  std::shared_ptr<DependencyInjector> injector_;
};

#define DECLARE_STAGE(NAME, CONTEXT)                          \
  class NAME : public Stage {                                 \
   public:                                                    \
    explicit NAME(const ScenarioConfig::StageConfig& config)  \
        : Stage(config) {}                                    \
    Stage::StageStatus Process(                               \
        const ::common::TrajectoryPoint& planning_init_point, \
        Frame* frame) override;                               \
    CONTEXT* GetContext() { return GetContextAs<CONTEXT>(); } \
  }

}  // namespace scenario
}  // namespace planning
}  // namespace zark
