/******************************************************************************
 * Copyright 2023 The zpilot . All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <memory>

#include "apps/planning/src/common/frame.h"
#include "status.h"
#include "apps/planning/src/tasks/task.h"

namespace zark {
namespace planning {

class Decider : public Task {
 public:
  explicit Decider(const TaskConfig& config);
  Decider(const TaskConfig& config,
          const std::shared_ptr<DependencyInjector>& injector);
  virtual ~Decider() = default;

  ::common::Status Execute(Frame* frame) override;

 protected:
  virtual ::common::Status Process(Frame* frame) {
    return ::common::Status::OK();
  }
};

}  // namespace planning
}  // namespace zark
