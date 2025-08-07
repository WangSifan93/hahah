/******************************************************************************
 * Copyright 2023 The zpilot Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>

#include "apps/planning/src/common/dependency_injector.h"
#include "apps/planning/src/common/frame.h"
#include "status.h"
#include "apps/planning/src/planning_msgs/planning_config.h"

namespace zark {
namespace planning {
using ::common::Status;
class Task {
 public:
  explicit Task(const TaskConfig& config);

  Task(const TaskConfig& config,
       const std::shared_ptr<DependencyInjector>& injector);

  virtual ~Task() = default;

  const std::string& Name() const;

  const TaskConfig& Config() const { return config_; }

  virtual Status Execute(Frame* frame);

 protected:
  Frame* frame_ = nullptr;

  TaskConfig config_;
  std::string name_;

  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace zark
