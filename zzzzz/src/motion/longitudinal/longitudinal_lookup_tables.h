/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file longitudinal_lookup_tables.h
 **/

#pragma once

#include "apps/planning/src/common/lookup_table.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

class LongitudinalLookupTables {
 public:
  explicit LongitudinalLookupTables(const LongitudinalOptimizerConfig& config);

  LookupTable a_accel_table() const { return a_accel_table_; }

  LookupTable front_soft_padding_table() const {
    return front_soft_padding_table_;
  }

  LookupTable rear_soft_padding_table() const {
    return rear_soft_padding_table_;
  }

  LookupTable a_max_soft_table() const { return a_max_soft_table_; }

  LookupTable a_max_stiff_table() const { return a_max_stiff_table_; }

  LookupTable a_min_soft_table() const { return a_min_soft_table_; }

  LookupTable front_padding_relative_speed_multiplier_table() const {
    return front_padding_relative_speed_multiplier_table_;
  }

  LookupTable rear_padding_relative_speed_multiplier_table() const {
    return rear_padding_relative_speed_multiplier_table_;
  }

  LookupTable front_padding_speed_tolerance_table() const {
    return front_padding_speed_tolerance_table_;
  }

  LookupTable rear_padding_speed_tolerance_table() const {
    return rear_padding_speed_tolerance_table_;
  }

  LongitudinalOptimizerConfig::Constraint::StiffPaddingTable
  front_stiff_padding_table() const {
    return config_.con.front_stiff_padding_table;
  }

  LongitudinalOptimizerConfig::Constraint::StiffPaddingTable
  rear_stiff_padding_table() const {
    return config_.con.rear_stiff_padding_table;
  }

 private:
  const LongitudinalOptimizerConfig config_;
  const LookupTable a_accel_table_;
  const LookupTable front_soft_padding_table_;
  const LookupTable rear_soft_padding_table_;
  const LookupTable a_max_soft_table_;
  const LookupTable a_max_stiff_table_;
  const LookupTable a_min_soft_table_;
  const LookupTable front_padding_relative_speed_multiplier_table_;
  const LookupTable rear_padding_relative_speed_multiplier_table_;
  const LookupTable front_padding_speed_tolerance_table_;
  const LookupTable rear_padding_speed_tolerance_table_;
};

}  // namespace planning
}  // namespace zark
