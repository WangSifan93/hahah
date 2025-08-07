/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file longitudinal_blocker.h
 **/

#pragma once

#include <vector>

#include "apps/planning/src/common/obstacle.h"
#include "apps/planning/src/common/proposal.h"
#include "apps/planning/src/motion/longitudinal/data_type.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

class LongitudinalBlocker {
 public:
  LongitudinalBlocker() = default;

  LongitudinalBlocker(LongitudinalOptimizerConfig::Model& config);

  /**
   * @brief Construct Blocker with STProposalConfig
   * @param st_proposal
   * @param obstacle_map
   * @param corridor
   * @return std::vector<Blocker>
   */
  std::vector<Blocker> ConstructBlocker(
      const STProposal& st_proposal,
      const IndexedPtrList<std::string, const Obstacle*>& obstacle_map,
      const Corridor& corridor);

  /**
   * @brief get lon speed with obstacle and corridor point
   * @param obs_point obstacle point
   * @param corr_point corridor point
   * @return double lon speed [m/s]
   */
  double ComputeLonSpeed(const ::common::TrajectoryPoint& obs_point,
                         const CorridorPoint& corr_point);

 private:
  LongitudinalOptimizerConfig::Model config_;
};

}  // namespace planning
}  // namespace zark
