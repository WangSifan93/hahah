/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file lateral_padding.cc
 **/

#include "apps/planning/src/motion/lateral/lateral_padding.h"

namespace zark {
namespace planning {

const double LateralPadding::GetStiffPadding(
    const LateralLookupTables& lookup_tables, const perception::SubType& type,
    const bool is_curb) const {
  const LateralOptimizerConfig::Constraint::StiffPaddingTable&
      stiff_padding_table = lookup_tables.stiff_padding_table();
  if (is_curb) {
    return stiff_padding_table.curb;
  }
  switch (type) {
    case perception::ST_BIG_BUS:
    case perception::ST_LIGHT_TRUCK:
    case perception::ST_HEAVY_TRUCK:
    case perception::ST_SMALL_BUS:
      return stiff_padding_table.large_car;
      break;
    case perception::ST_CAR:
      return stiff_padding_table.regular_car;
      break;
    case perception::ST_CYCLIST:
    case perception::ST_MOTORCYCLIST:
      return stiff_padding_table.bicycle;
      break;
    case perception::ST_PEDESTRIAN:
      return stiff_padding_table.pedestrian;
    default:
      return stiff_padding_table.others;
      break;
  }
}

const double LateralPadding::GetSoftPadding(
    const LateralLookupTables& lookup_tables, const double v_obs) const {
  return lookup_tables.soft_padding_table().Evaluate(v_obs);
}

}  // namespace planning
}  // namespace zark
