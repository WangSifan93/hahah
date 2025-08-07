/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_tube.h
 **/

#pragma once

#include <string>
#include <vector>

#include "apps/planning/src/common/corridor.h"
#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/lookup_table.h"
#include "apps/planning/src/common/mpc_data.h"
#include "apps/planning/src/motion/lateral/data_type.h"
#include "apps/planning/src/motion/lateral/lateral_padding.h"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "apps/planning/src/decision/longitudinal/data_type.h"

namespace zark {
namespace planning {

/**
 * class LateralTube
 * @brief build lateral tube with corridor and lon mpc data.
 */
class LateralTube {
 public:
  LateralTube(const LateralOptimizerConfig& config) { config_ = config; }
  /**
   * @brief Build tube structure.
   *
   * @param planning_start_point planning start point
   * @param nudgers lateral nudgers
   * @param corridor_info corridor data
   * @param lon_mpc_data lon mpc data
   * @param corridor_prev corridor of previous frame
   * @param tube_prev tube of previous frame
   * @param lateral_tables lateral tables from config
   * @param lateral_padding lateral padding from config
   * @param st_proposal st proposal
   * @param cur_local_route current local route
   * @return tube lateral tube
   */
  Tube ConstructTubes(const ::common::TrajectoryPoint& planning_start_point,
                      const std::vector<Nudger>& nudgers,
                      const CorridorInfo& corridor_info,
                      const LonMPCData& lon_mpc_data,
                      const Corridor& corridor_prev,
                      const std::vector<Tube::TubePoint>& tube_prev,
                      const LateralLookupTables& lateral_tables,
                      const LateralPadding& lateral_padding,
                      const STProposal& st_proposal,
                      const LocalRoute& cur_local_route) const;

 private:
  /**
   * @brief Convert SL coordinates to XY coordinates for points in the Tube
   * object.
   *
   * @param tube Reference to the Tube object whose points' coordinates will be
   * converted.
   * @param corridor Reference to the Corridor object used for SL to XY
   * conversion.
   */
  void ConvertVec2dPoint(Tube& tube, const Corridor& Corridor) const;

  /**
   * @brief Processes the tube reference points.
   *
   * @param pts A vector of Tube::TubePoint objects representing the reference
   * points of the tube.
   * @param is_left A boolean flag indicating whether the tube is on the left
   * side (true) or not (false).
   */
  void SmoothTubeReference(std::vector<Tube::TubePoint>& pts,
                           const bool is_left) const;

  /**
   * @brief Evaluate the tube point at a given length 's'.
   *
   * @param tube vector of tube points for linear interpolation
   * @param s TubePoint in s.
   * @return TubePoint at the given length 's'.
   */
  Tube::TubePoint EvaluateByS(const std::vector<Tube::TubePoint>& tube,
                              const double s) const;

  /**
   * @brief Find the bottom left point.
   *
   * @param st_proposals A vector of CorridorInfo::STProposal representing the
   * st proposals.
   */
  std::optional<STPoint> FindBottomLeftPoint(
      const STProposal& st_proposal) const;

  /**
   * @brief Get corridor axis lanes width
   *
   * This function calculates and updates the lane left boundary and right
   * boundary based on the given corridor point and local route.
   *
   * @param corridor_point The corridor point, containing relevant information
   * about the corridor
   * @param cur_local_route The current local route, containing relevant
   * information about the road
   * @param lanes_left_boundary The lane left boundary, the function will update
   * the value of this parameter
   * @param lanes_right_boundary The lane right boundary, the function will
   * update the value of this parameter
   */
  void GetCorridorAxisLanesWidth(const CorridorPoint& corridor_point,
                                 const LocalRoute& cur_local_route,
                                 double& lanes_left_boundary,
                                 double& lanes_right_boundary) const;

 private:
  FRIEND_TEST(LateralTubeTest, TestGetCorridorAxisLanesWidth);
  FRIEND_TEST(LateralTubeTest, TestSmoothTubeReference);
  FRIEND_TEST(LateralTubeTest, TestEvaluteByS);
  FRIEND_TEST(LateralTubeTest, TestFindBottomLeftPoint);

 private:
  LateralOptimizerConfig config_;
};

}  // namespace planning
}  // namespace zark
