/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file lateral_padding.h
 **/

#pragma once

#include "apps/planning/src/common/perception_obstacle.h"
#include "apps/planning/src/motion/lateral/lateral_lookup_tables.h"

namespace zark {
namespace planning {

class LateralPadding {
 public:
  LateralPadding() = default;

  /**
   * @brief get stiff padding.
   * @param lookup_tables lateral lookup tables
   * @param type obstacle type
   * @param is_curb is curb
   * @return stiff padding [m]
   */
  const double GetStiffPadding(const LateralLookupTables& lookup_tables,
                               const perception::SubType& type,
                               const bool is_curb = false) const;

  /**
   * @brief get soft padding.
   * @param lookup_tables lateral lookup tables
   * @param v_obs obstacle velocity [m/s]
   * @return soft padding [m]
   */
  const double GetSoftPadding(const LateralLookupTables& lookup_tables,
                              const double v_obs) const;
};

}  // namespace planning
}  // namespace zark
