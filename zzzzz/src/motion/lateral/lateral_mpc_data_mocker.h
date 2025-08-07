/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_mpc_data_mocker.h
 **/

#pragma once

#include <vector>

#include "Eigen/Core"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "apps/planning/src/common/mpc_data.h"
#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/mpc_data.h"
#include "apps/planning/src/motion/lateral/lateral_reference.h"

namespace zark {
namespace planning {

/**
 * class LateralMPCDataMocker
 * @brief Mocker lat mpc data by corridor info.
 */
class LateralMPCDataMocker {
 public:
  LateralMPCDataMocker() = default;

  LateralMPCDataMocker(LateralOptimizerConfig& config);

  /**
   * @brief Generate fake lateral MPC data
   *
   * @param corridor_info corridor data
   * @param lon_mpc_data lon mpc data
   * @param lat_ref lateral reference
   *
   * @return LatMPCData
   */
  LatMPCData GenerateLateralMPCData(const CorridorInfo& corridor_info,
                                    const LonMPCData& lon_mpc_data,
                                    LateralReference* lat_ref) const;

 private:
  /**
   * @brief Generate fake lateral MPC data
   *
   * @param corridor_info corridor data
   * @param lon_mpc_data lon mpc data
   *
   * @return Tube
   */
  Tube GenerateEmptyTube(const CorridorInfo& corridor_info,
                         const LonMPCData& lon_mpc_data) const;

 private:
  FRIEND_TEST(LateralMPCDataMockerTest, TestGenerateEmptyTube);

 private:
  LateralOptimizerConfig config_;
};
}  // namespace planning
}  // namespace zark
