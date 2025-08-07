/******************************************************************************
 * Copyright 2024 The zark Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lc_request.h
 **/

#pragma once

#include "apps/planning/src/common/dependency_injector.h"
#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/common/macros.h"
#include "apps/planning/src/common/mission.h"
#include "apps/planning/src/mission/force_lane_change_decision.h"
#include "messages/planning/ads_decision.pb.h"
#include "messages/state_management/state_management.pb.h"

namespace zark {
namespace planning {

/**
 * @brief LCRequest:use this class to calculate lc request(mission)
 *
 */
class LCRequest {
 public:
  LCRequest(const MissionDeciderConfig& config);
  ~LCRequest();

  enum EfficientLane {
    CUR_LANE = 0,
    LEFT_LANE = 1,
    RIGHT_LANE = 2,
  };

  /**
   * @brief Lane Change Request Logic process
   *
   * @param frame
   * @return Mission
   */
  Mission Process(Frame* frame);

  bool GetIsNearRamp();

 private:
  /**
   * @brief Target Local Route Logic Process
   *
   * @param mission
   * @param current_local_route
   */
  void UpdateTargetLocalRoute(const bool use_left_refline,
                              const bool use_right_refline,
                              LocalRoute& current_local_route);

  /**
   * @brief Cross lane line  check Logic Process
   *
   * @param fct_lc_state mcu lane change machine state
   * @param fitted_result include local route cross line signal
   * @param arriving_target_line
   * @return return true if crossed lane line, else return false.
   */
  bool CheckCrossedLine(const ads_common::eALCS_ALCState& fct_lc_state,
                        const LocalRoute& current_local_route,
                        bool& arriving_target_line);

 private:
  HazardVehInfo hazard_veh_info_;
  EfficentLaneInfo efficent_lane_info_;

  double default_urgency_ = 0.0;

  MissionDeciderConfig config_;
  Mission mission_;

  bool arriving_target_line_;

 private:
  FRIEND_TEST(LCRequestTest, TestManualLCCheck);
  FRIEND_TEST(LCRequestTest, TestForceLCCheck);
  FRIEND_TEST(LCRequestTest, TestRouteLCCheck);
  FRIEND_TEST(LCRequestTest, TestProcBlockingLC);
  FRIEND_TEST(LCRequestTest, TestEfficientLCCheck);
  FRIEND_TEST(LCRequestTest, TestUpdateHazardVeh);
  FRIEND_TEST(LCRequestTest, TestUpdateEfficentLaneActive);
  FRIEND_TEST(LCRequestTest, TestGetLCReqFromHazardVeh);
  FRIEND_TEST(LCRequestTest, TestGetLCReqFromEfficientInfo);
};

}  // namespace planning
}  // namespace zark
