/******************************************************************************
 * Copyright 2023 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file msission_decider.h
 **/

#pragma once

#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/common/macros.h"
#include "apps/planning/src/mission/env_info_builder.h"
#include "apps/planning/src/mission/lc_request.h"
#include "apps/planning/src/tasks/deciders/decider.h"

namespace zark {
namespace planning {

class MissionDecider : public Decider {
 public:
  /**
   * @brief Construct a new Mission Decider object
   *
   * @param config
   * @param injector
   */
  MissionDecider(const TaskConfig& config,
                 const std::shared_ptr<DependencyInjector>& injector);

  /**
   * @brief MissionDecider execute function
   *
   * @param frame
   * @return ::common::Status
   */
  ::common::Status Execute(Frame* frame);

 private:
  /**
   * @brief Set the Target Local Route object
   *
   * @param frame
   */
  void SetTargetLocalRoute(Frame* frame);

 private:
  std::unique_ptr<EnvInfoBuilder> env_info_builder_;
  std::unique_ptr<LCRequest> lc_request_;
};

}  // namespace planning
}  // namespace zark
