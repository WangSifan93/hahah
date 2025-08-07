/******************************************************************************
 * Copyright 2017 The zpilot Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <list>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "apps/planning/src/common/indexed_queue.h"
#include "apps/planning/src/common/local_view.h"
#include "apps/planning/src/common/mission.h"
#include "apps/planning/src/common/obstacle.h"
#include "apps/planning/src/common/proposal.h"
#include "apps/planning/src/common/data_reader/data_reader.h"
#include "apps/planning/src/common/proto_convertor/proto_convertor.h"
#include "apps/planning/src/common/trajectory/publishable_trajectory.h"
#include "apps/planning/src/common/vehicle_state/proto/vehicle_state.h"
#include "apps/planning/src/common/vehicle_state/vehicle_state_provider.h"
#include "apps/planning/src/common/proto_convertor/proto_convertor.h"
#include "apps/planning/src/common/data_reader/forcelc_measure.h"
#include "apps/planning/src/planning_msgs/planning.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "geometry.h"
#include "status.h"
#include "vec2d.h"

namespace zark {
namespace planning {

/**
 * @brief The vehicle mode state
 *
 */
enum class AutoMode {
  MANUAL = 0,
  AUTO_FULL = 1,
  AUTO_LON = 2,
};

/**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.
 */

class Frame {
 public:
  explicit Frame(uint32_t sequence_num);

  Frame(uint32_t sequence_num, const LocalView &local_view,
        const ::common::TrajectoryPoint &planning_start_point,
        const common::VehicleState &vehicle_state);

  virtual ~Frame() = default;

  const ::common::TrajectoryPoint &PlanningStartPoint() const;

  ::common::Status Init(
      const common::VehicleStateProvider *vehicle_state_provider,
      const std::vector<routing::LaneWaypoint> &future_route_waypoints);

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory &ComputedTrajectory() const;

  const std::list<LocalRoute> &LocalRoutes() const;

  inline std::list<LocalRoute> &MutableLocalRoutes() { return local_routes_; }

  LocalRoute *FindCurrentLocalRoute();

  void SetTargetLocalRoute(LocalRoute *local_route) {
    target_local_route_ = local_route;
  }

  const LocalRoute *GetTargetLocalRoute() { return target_local_route_; }

  Obstacle *Find(const std::string &id);

  const std::vector<const Obstacle *> obstacles() const;

  const common::VehicleState &vehicle_state() const;

  static void AlignPredictionTime(
      const double planning_start_time,
      zark::prediction::proto::PredictionObjects *prediction_obstacles);

  void set_current_frame_planned_trajectory(
      const ADCTrajectory &current_frame_planned_trajectory) {
    current_frame_planned_trajectory_ = current_frame_planned_trajectory;
  }

  const ADCTrajectory &current_frame_planned_trajectory() const {
    return current_frame_planned_trajectory_;
  }

  void set_planning_start_point(
      const ::common::TrajectoryPoint &planning_start_point) {
    planning_start_point_ = planning_start_point;
  }

  const double ObstacleTimeHorizon() const { return obstacle_time_horizon_; }

  const LocalView &local_view() const { return local_view_; }

  inline void set_local_view(const LocalView &local_view) {
    local_view_ = local_view;
  }

  const std::vector<CorridorInfo> &GetCorridorInfos() const {
    return corridors_infos_;
  }

  std::vector<CorridorInfo> *MutableCorridorInfos() {
    return &corridors_infos_;
  }

  const std::vector<Proposal> &GetProposals() const { return proposals_; }

  std::vector<Proposal> *MutableProposals() { return &proposals_; }

  ADCTrajectory *MutableCurrentFramePlannedTrajectory() {
    return &current_frame_planned_trajectory_;
  }

  const Mission &GetMission() const { return mission_; }

  void SetMission(const Mission &mission) { mission_ = mission; }

  const ForceLcMeasureInfo &GetNoaDecisionMeas() {
    return decision_measure_info_;
  }

  void SetNoaDecisionMeas(const ForceLcMeasureInfo &measure_info) {
    decision_measure_info_ = measure_info;
  }

  const ads_decision::DEC_Outputs &GetFittedLaneLine() {
    return fitted_lane_line_;
  }

  void SetFittedLaneLine(const ads_decision::DEC_Outputs &result) {
    fitted_lane_line_ = result;
  }

  void AddObstacle(const Obstacle &obstacle);

  inline const bool IsHdMap() const { return is_hdmap_; }

  const bool IsReplan() const { return is_replan_; }

  const bool IsLatReplan() const { return is_lat_replan_; }

  const bool IsInAutoMode() const { return is_in_auto_state_; }

  void SetIsInAutoState(const bool &is_in_auto_state) {
    is_in_auto_state_ = is_in_auto_state;
  }

  const bool IsNearIntersection() const { return is_near_intersection_; }

  void SetIsReplan(const bool &is_replan) { is_replan_ = is_replan; }

  void SetIsLatReplan(const bool &is_lat_replan) { is_lat_replan_ = is_lat_replan; }

  inline void SetIsNearIntersection(const bool is_near_intersection) {
    is_near_intersection_ = is_near_intersection;
  }

  const GapChoiceInfo GetLCGap() const { return lc_gap_; }

  void SetLCGap(const GapChoiceInfo &lc_gap) { lc_gap_ = lc_gap; }

  /**
   * @brief Set AutoMode
   *
   */
  void SetAutoMode(const AutoMode auto_mode) { auto_mode_ = auto_mode; };

  /**
   * @brief Get the Driving Mode object
   *
   * @return AutoMode
   */
  AutoMode GetAutoMode() const { return auto_mode_; }

  /**
   * @brief Set the Ies State object
   *
   */
  void SetIsInIESMode(bool is_in_ies_mode) { is_in_ies_mode_ = is_in_ies_mode; }

  /**
   * @brief Get the Ies State object
   *
   */
  bool GetIsInIESMode() const { return is_in_ies_mode_; }

 private:
  ::common::Status InitFrameData(
      const common::VehicleStateProvider *vehicle_state_provider);

  void AddObstacleRegionType(Obstacle &obstacle,
                             const LocalRoute &current_local_route);

  void ReadPadMsgDrivingAction();
  void ResetPadMsgDrivingAction();

  /**
   * @brief update Left Local Route
   *
   */
  void UpdateLeftLocalRoute();

  /**
   * @brief update Right Local Route
   *
   */
  void UpdateRightLocalRoute();

  void SetIsOncoming(Obstacle &obstacle,
                     const VehicleState &vehicle_state) const;
                     
  void SetIsApproachingStop(Obstacle &obstacle) const;

 private:
  bool is_hdmap_;
  bool is_replan_ = false;
  bool is_lat_replan_ = false;
  bool is_in_auto_state_ = false;
  bool is_near_intersection_ = false;

  GapChoiceInfo lc_gap_;
  uint32_t sequence_num_ = 0;
  LocalView local_view_;
  ::common::TrajectoryPoint planning_start_point_;
  common::VehicleState vehicle_state_;
  std::list<LocalRoute> local_routes_;
  LocalRoute *target_local_route_;
  Mission mission_;
  ForceLcMeasureInfo decision_measure_info_;
  ads_decision::DEC_Outputs fitted_lane_line_;
  std::vector<CorridorInfo> corridors_infos_;
  std::vector<Proposal> proposals_;

  ThreadSafeIndexedObstacles obstacles_;

  ADCTrajectory current_frame_planned_trajectory_;

  double obstacle_time_horizon_ = 8.5;  // [s]

  AutoMode auto_mode_ = AutoMode::MANUAL;
  bool is_in_ies_mode_ = false;

 private:
  FRIEND_TEST(FrameTest, TestCurrentLaneObstacleFilter);
  FRIEND_TEST(FrameTest, TestLeftLaneObstacleFilter);
  FRIEND_TEST(FrameTest, TestRightLaneObstacleFilter);
  FRIEND_TEST(FrameTest, TestLeftLeftLaneObstacleFilter);
  FRIEND_TEST(FrameTest, TestRightRightLaneObstacleFilter);
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 public:
  FrameHistory();
};

}  // namespace planning
}  // namespace zark
