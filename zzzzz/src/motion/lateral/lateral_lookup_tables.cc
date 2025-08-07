/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file lateral_lookup_tables.cc
 **/

#include "apps/planning/src/motion/lateral/lateral_lookup_tables.h"

namespace zark {
namespace planning {

LateralLookupTables::LateralLookupTables(const LateralOptimizerConfig& config)
    : config_(config),
      soft_padding_table_(LookupTable(config.con.soft_padding_table)) {}

}  // namespace planning
}  // namespace zark
