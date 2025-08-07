/******************************************************************************
 * Copyright 2024 The zark Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file env_info_builder.h
 **/

#pragma once

#include "apps/planning/src/common/mission.h"
#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/common/macros.h"
#include "apps/planning/src/planning_msgs/planning_config.h"

namespace zark {
namespace planning {

using VehicleState = common::VehicleState;

/**
 * @brief EnvInfoBuilder:use this class to get all env infos
 *
 */
class EnvInfoBuilder {
 public:
  EnvInfoBuilder(const MissionDeciderConfig& config);
  ~EnvInfoBuilder();

  EnvInfos Build(const common::VehicleState& vehicle_state,
                 const double driver_set_v,
                 const std::vector<const Obstacle*>& obstacles,
                 const LocalRoute& current_local_route);

 private:
  /**
   * @brief Update Vehicle Speed Infos,vehicle current speed and driver set
   * speed
   *
   * @param vehicle_state
   * @param v_set
   * @return VehicleInfos
   */
  VehicleInfos UpdateVehicleInfo(const VehicleState& vehicle_state,
                                 const double& v_set);

  /**
   * @brief Update Lane Infos, current,left,right local route, lane width,lane
   * type etc.
   *
   * @param vehicle_state
   * @param cur_local_route
   * @return LaneInfos
   */
  LaneInfos UpdateLaneInfo(const VehicleState& vehicle_state,
                           const LocalRoute& cur_local_route);

  /**
   * @brief Update Obstacles Infos,find current lane,left lane,right lane,lleft
   * lane,rright lane obstacles
   *
   * @param obstacles
   * @param lane_infos
   * @return ObstacleInfos
   */
  ObstacleInfos UpdateObstaclesInfo(
      const std::vector<const zark::planning::Obstacle*>& obstacles,
      const LaneInfos& lane_infos);

  /**
   * @brief find whether current lane is blocked
   *
   * @param obs_info
   * @param lane_infos
   * @return CurLaneBlockInfos
   */
  CurLaneBlockInfos IsCurrentLaneBlock(const ObstacleInfos& obs_info,
                                       const LaneInfos& lane_infos);

  /**
   * @brief update current,left,right lane pass speed and speed limit infos
   *
   * @param obs_info
   * @param lane_infos
   * @return LaneSpdInfos
   */
  LaneSpdInfos UpdateLaneSpdInfo(const ObstacleInfos& obs_info,
                                 const LaneInfos& lane_infos);

  /**
   * @brief update current lane fury value,if front obstacle is moving slow
   *
   * @param lane_spd_info
   * @return double(fury value)
   */
  double UpdateFuryVal(const LaneSpdInfos& lane_spd_info);

  /**
   * @brief update route lane change params according to vehicle speed and some
   * preset params
   *
   * @param lane_infos
   * @param lane_spd_info
   * @return RCParamInfos
   */
  RCParamInfos UpdateRouteLCParams(const LaneInfos& lane_infos,
                                   const LaneSpdInfos& lane_spd_info);

  /**
   * @brief update Lane Change Inhibit Infos
   *
   * @param lane_infos
   * @return InhibitInfos
   */
  InhibitInfos UpdateLCInhibInfos(const VehicleInfos& vehicle_infos,
                                  const LaneInfos& lane_infos);

  /**
   * @brief check the obstacle need ignore
   *
   * @param obs_id
   * @return true if the obstacle need ignored, otherwise false
   */
  bool ObstacleNeedIgnore(const std::string& obs_id);

  /**
   * @brief find the nearest obstalce index according to adc_s
   *
   * @param adc_s
   * @param obj_vec
   * @return int8_t
   */
  int8_t FindNearestObjIndex(const double& adc_s,
                             const std::vector<const Obstacle*>& obj_vec);

  /**
   * @brief compare two obstacle with obstacle boundary start s
   *
   * @param Obstacle a
   * @param Obstacle b
   * @return true if a boundary start s < b boundary start s,otherwise return
   * false
   */
  static bool CompareByStarts(const Obstacle* a, const Obstacle* b) {
    return a->PerceptionSLBoundary().start_s() <
           b->PerceptionSLBoundary().start_s();
  }

  /**
   * @brief linear interpretation
   *
   * @param x
   * @param y
   * @param xi
   * @return double
   */
  double LinearInterp(const std::vector<double>& x,
                      const std::vector<double>& y, const double xi);

 private:
  uint8_t blocking_cnt_ = 0;
  uint8_t false_blocking_cnt_ = 0;

  MissionDeciderConfig config_;

 private:
  FRIEND_TEST(EnvInfoBuilderTest, TestUpdateVehicleInfo);
  FRIEND_TEST(EnvInfoBuilderTest, TestUpdateLaneInfo);
  FRIEND_TEST(EnvInfoBuilderTest, TestUpdateObstaclesInfo);
  FRIEND_TEST(EnvInfoBuilderTest, TestIsCurrentLaneBlock);
  FRIEND_TEST(EnvInfoBuilderTest, TestUpdateLaneSpdInfo);
  FRIEND_TEST(EnvInfoBuilderTest, TestUpdateFuryVal);
  FRIEND_TEST(EnvInfoBuilderTest, TestUpdateRouteLCParams);
};

}  // namespace planning
}  // namespace zark
