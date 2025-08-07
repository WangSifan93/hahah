/******************************************************************************
 * Copyright 2024 The zpilot Authors fuhh. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file lookup_table.h
 **/

#pragma once

#include <vector>

#include "apps/planning/src/planning_msgs/planning_config.h"

namespace zark {
namespace planning {
class LookupTable {
 public:
  LookupTable() = default;

  LookupTable(const LookupTableConfig& config);

  /**
   * @brief evaluate linear result
   * @param x
   * @return linear interpolation result
   */
  double Evaluate(const double x) const;

 private:
  /**
   * @brief check lookup table is valid
   * @param config
   * @return result
   */
  bool CheckInputValid(const LookupTableConfig& config);

 private:
  LookupTableConfig config_;
};

}  // namespace planning
}  // namespace zark
