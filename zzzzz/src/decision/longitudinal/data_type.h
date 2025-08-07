/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file: data_type.h
 **/

#pragma once

#include <vector>

#include "apps/planning/src/common/speed/st_boundary.h"

namespace zark {
namespace planning {

using STGraph = std::vector<STBoundary>;
using STProposal = std::vector<std::tuple<const STBoundary*,
                                          bool,    // is_front
                                          bool>>;  // is_filtered
using STTopology = std::vector<std::vector<const STBoundary*>>;

}  // namespace planning
}  // namespace zark
