/******************************************************************************
 * Copyright 2023 The zpilot Author Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

#include "apps/planning/src/planning_base.h"

#include "apps/planning/src/common/clock.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/tasks/task_factory.h"

namespace zark {
namespace planning {

PlanningBase::PlanningBase(const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

PlanningBase::~PlanningBase() {}

Status PlanningBase::Init(const PlanningConfig& config) {
  TaskFactory::Init(config, injector_);
  return Status::OK();
}

}  // namespace planning
}  // namespace zark
