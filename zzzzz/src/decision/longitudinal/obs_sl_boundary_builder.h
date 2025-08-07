/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file obs_sl_boundary_builder.h
 **/

#pragma once

#include <unordered_map>
#include <vector>

#include "apps/planning/src/common/corridor.h"
#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/obstacle.h"
#include "apps/planning/src/planning_msgs/sl_boundary.h"

namespace zark {
namespace planning {

/**
 * class ObsSLBoundaryBuilder
 * @brief Build SL boundaries map for all obstacles with corridor points.
 */
class ObsSLBoundaryBuilder {
 public:
  ObsSLBoundaryBuilder() = default;

  /**
   * @brief Build SL boundaries map for all obstacles with corridor points.
   *
   * @param corridor A list of corridor points.
   * @param obstacle_map A list of obstacles.
   * @return std::unordered_map<const Obstacle*, SLTrajectory> An map
   * which include pointer of each obstacle and it's SL boundaries.
   */
  std::unordered_map<const Obstacle*, SLTrajectory> BuildObsSLBoundaryMap(
      const Corridor& corridor,
      const std::vector<const Obstacle*>& obstacle_map);

  /**
   * @brief Compute SL boundaries for an obstacle with corridor points.
   *
   * @param corridor A list of corridor points.
   * @param obstacle An obstacle.
   * @return SLTrajectory A list of SL boundaries.
   */
  SLTrajectory ComputeObstacleSLBoundary(const Corridor& corridor,
                                         const Obstacle* obstacle);
};

}  // namespace planning
}  // namespace zark
