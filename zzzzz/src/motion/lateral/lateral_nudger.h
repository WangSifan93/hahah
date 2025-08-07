/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_nudger.h
 **/

#pragma once

#include <string>
#include <vector>

#include "box2d.h"
#include "vec2d.h"
#include "apps/planning/src/common/corridor.h"
#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/mpc_data.h"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "apps/planning/src/common/obstacle.h"

namespace zark {
namespace planning {

/**
 * class LateralNudger
 * @brief build lateral nudger with corridor and lon mpc data.
 */
class LateralNudger {
 public:
  LateralNudger(const LateralOptimizerConfig& config) { config_ = config; }

  /**
   * @brief Build nudger structure.
   *
   * @param lon_mpc_data lon mpc data
   * @param lateral_obstacles obstacle info get from st graph builder
   * @param corridor_info corridor info
   * @return nudgers
   */
  std::vector<Nudger> ConstructNudger(
      const std::vector<std::pair<const Obstacle*, bool>>& lateral_obstacles,
      const LonMPCData& lon_mpc_data, const CorridorInfo& corridor_info) const;

  /**
   * @brief Check the overlap form corridor point to obstacle trajectory point.
   *
   * @param s s in corridor
   * @param corridor corridor in order to use SLTOXY function
   * @param obs_sl_boundary obstacle sl boundaries
   * @param is_left obstacle is located on left side of the corridor
   * @return whether ADC bounding box overlaps with obstacle bounding box
   */  
  bool NeedUpdateNudger(const double& s, const Corridor& corridor,
                        const SLBoundary& obs_sl_boundary, bool is_left) const;
 private:
  LateralOptimizerConfig config_;
};

}  // namespace planning
}  // namespace zark
