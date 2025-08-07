/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file longitudinal_lookup_tables.cc
 **/

#include "apps/planning/src/motion/longitudinal/longitudinal_lookup_tables.h"

namespace zark {
namespace planning {

LongitudinalLookupTables::LongitudinalLookupTables(
    const LongitudinalOptimizerConfig& config)
    : config_(config),
      a_accel_table_(LookupTable(config.ref.a_accel_table)),
      front_soft_padding_table_(
          LookupTable(config.con.front_soft_padding_table)),
      rear_soft_padding_table_(LookupTable(config.con.rear_soft_padding_table)),
      a_max_soft_table_(LookupTable(config.con.a_max_soft_table)),
      a_max_stiff_table_(LookupTable(config.con.a_max_stiff_table)),
      a_min_soft_table_(LookupTable(config.con.a_min_soft_table)),
      front_padding_relative_speed_multiplier_table_(LookupTable(
          config.con.front_padding_relative_speed_multiplier_table)),
      rear_padding_relative_speed_multiplier_table_(
          LookupTable(config.con.rear_padding_relative_speed_multiplier_table)),
      front_padding_speed_tolerance_table_(
          LookupTable(config.con.front_padding_speed_tolerance_table)),
      rear_padding_speed_tolerance_table_(
          LookupTable(config.con.rear_padding_speed_tolerance_table)) {}

}  // namespace planning
}  // namespace zark
