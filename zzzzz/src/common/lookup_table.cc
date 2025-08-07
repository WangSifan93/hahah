/******************************************************************************
 * Copyright 2024 The zpilot Authors fuhh. All Rights Reserved.
 *
 *****************************************************************************/

#include "apps/planning/src/common/lookup_table.h"

#include "linear_interpolation.h"
#include "apps/planning/src/common/log.h"

namespace zark {
namespace planning {

LookupTable::LookupTable(const LookupTableConfig& config) {
  config_ = config;
  if (!CheckInputValid(config)) {
    AFATAL << "The lookup table config is NOT monotonic!";
  }
}

bool LookupTable::CheckInputValid(const LookupTableConfig& config) {
  size_t point_num = config.points_x.size();
  if (config.points_y.size() != point_num) {
    AINFO << " input x size is not equal to y size"
          << " x size = " << point_num
          << ", y size = " << config.points_y.size();
    return false;
  }
  if (point_num < 1) {
    AINFO << "Table is empty";
    return false;
  }
  for (size_t i = 1; i < point_num; i++) {
    if (config.points_x.at(i - 1) > config.points_x.at(i)) {
      AINFO << "Table is not monotonic";
      return false;
    }
  }
  return true;
}

double LookupTable::Evaluate(const double x) const {
  return ::math::interp_1d(x, config_.points_x, config_.points_y);
}

}  // namespace planning
}  // namespace zark
