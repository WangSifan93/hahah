/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file lateral_lookup_tables.h
 **/

#pragma once

#include "apps/planning/src/common/lookup_table.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

class LateralLookupTables {
 public:
  explicit LateralLookupTables(const LateralOptimizerConfig& config);

  LateralOptimizerConfig::Constraint::StiffPaddingTable stiff_padding_table()
      const {
    return config_.con.stiff_padding_table;
  }

  LookupTable soft_padding_table() const { return soft_padding_table_; }

 private:
  LateralOptimizerConfig config_;
  LookupTable soft_padding_table_;
};

}  // namespace planning
}  // namespace zark
