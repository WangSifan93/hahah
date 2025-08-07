/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file corridor_info.cc
 **/

#include "apps/planning/src/common/corridor_info.h"

namespace zark {
namespace planning {

CorridorInfo::CorridorInfo(
    const LocalRoute& local_route, const Mission& mission,
    const CorridorInfo::Type& corridor_type, const Corridor& corridor,
    const int idx_start_point,
    const IndexedPtrList<std::string, const Obstacle*>& obstacle_map)
    : local_route_(local_route),
      mission_(mission),
      type_(corridor_type),
      corridor_(corridor),
      idx_start_point_(idx_start_point),
      obstacle_map_(obstacle_map) {}

}  // namespace planning
}  // namespace zark
