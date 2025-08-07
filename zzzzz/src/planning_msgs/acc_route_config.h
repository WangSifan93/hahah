/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <string>

namespace zark {
namespace planning {

class ACCTrajectoryBuilderConfig {
 public:
  void set_t_total(double t_total) { t_total_ = t_total; }

  double t_total() const { return t_total_; }

 private:
  double t_total_;
};

class ACCRouteConfig {
 public:
  void set_acc_trajectory_builder_config(
      ACCTrajectoryBuilderConfig acc_trajectory_builder_config) {
    acc_trajectory_builder_config_ = acc_trajectory_builder_config;
  }

  const ACCTrajectoryBuilderConfig &acc_trajectory_builder_config() const {
    return acc_trajectory_builder_config_;
  }

 private:
  ACCTrajectoryBuilderConfig acc_trajectory_builder_config_;
};
}  // namespace planning
}  // namespace zark
