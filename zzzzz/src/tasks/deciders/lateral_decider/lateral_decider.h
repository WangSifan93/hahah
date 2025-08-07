/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file lateral_decider.h
 **/

#include "apps/planning/src/common/corridor.h"
#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/lookup_table.h"
#include "apps/planning/src/common/mission.h"
#include "apps/planning/src/common/hysteresis.h"
#include "apps/planning/src/decision/lateral/frenet_trajectory_sampler.h"
#include "apps/planning/src/tasks/deciders/decider.h"
#include "pnc_point.h"
#include "vec2d.h"
#include "apps/planning/src/decision/lateral/corridor_info_builder.h"
#include "apps/planning/src/decision/lateral/frenet_trajectory_map_builder.h"

namespace zark {
namespace planning {

using FrenetTrajectory = std::vector<::common::FrenetPoint>;
using CartesianTrajectory = std::vector<::common::CartesianPoint>;
using TrajectoryMap =
    std::unordered_map<CorridorInfo::Type,
                       std::pair<CartesianTrajectory, double>>;

/**
 * class LateralDecider
 * @brief Lateral decider.
 */
class LateralDecider : public Decider {
 public:
  LateralDecider(const TaskConfig &config,
                 const std::shared_ptr<DependencyInjector> &injector);

  /**
   * @brief Execution function of lateral decider.
   *
   * @param frame Frame information.
   * @return Status Execution status.
   */
  ::common::Status Execute(Frame *frame) override;

 private:
  CorridorPoint ComputeCorridorStartPoint(const bool is_replan);

  /**
   * @brief Compute corridor based on frenet sampling trajectory.
   *
   * @param local_route Local route.
   * @param frenet_sampling_traj Frenet sampling trajectory.
   * @param type Corridor type.
   * @return Corridor Corridor computed.
   */
  Corridor ComputeCorridor(const ::common::FrenetPoint &init_frenet_point,
                           const LocalRoute &local_route,
                           const CartesianTrajectory &frenet_sampling_traj,
                           const double v_target, const int i_start_frenet_traj,
                           int &idx_start_point);

  /**
   * @brief Compute front and rear corridor segment.
   *
   * @param local_route Local route.
   * @param s_start_local_route S start in local route cooridinate.
   * @param s_start_corridor S start in corridor coordinate.
   * @param l L lateral position.
   * @param num_points Number of points in corridor.
   * @param corridor_points Converted corridor points.
   */
  void ComputeEndCorridorSegment(const LocalRoute &local_route,
                                 const double s_start_local_route,
                                 const double s_start_corridor, const double l,
                                 const double v, const double dt,
                                 const int num_points,
                                 std::vector<CorridorPoint> &corridor_points);

  /**
   * @brief Compute middle corridor segment.
   *
   * @param init_frenet_point init frenet point.
   * @param local_route Local route.
   * @param frenet_sampling_traj Cartesian sampling trajectory.
   * @param type Corridor type.
   * @param corridor_points Converted corridor points.
   */
  void ComputeMiddleCorridorSegment(
      const ::common::FrenetPoint &init_frenet_point,
      const LocalRoute &local_route, const CartesianTrajectory &traj_cartesian,
      const int i_start_cartesian_traj,
      std::vector<CorridorPoint> &corridor_points);

  /**
   * @brief Calculate l left and l right based on curb.
   *
   * @param local_route Local route.
   * @param s_ref s to calculate distance to curb.
   * @param l_ref l of corridor point in local route.
   * @param l_left l left [m].
   * @param l_right l right [m].
   */
  void CalcLLeftAndRightBasedOnCurb(const LocalRoute &local_route,
                                    const double s_ref, const double l_ref,
                                    double &l_left, double &l_right);

  /**
   * @brief filter obstacles by mission type.
   *
   * @param init_frenet_point init frenet point.
   * @param obstacles original obstacles.
   * @param lane_change_request mission request.
   * @param type corridor type
   * @return obstacle map.
   */
  IndexedPtrList<std::string, const Obstacle *> FilterObstacles(
      const ::common::FrenetPoint &init_frenet_point,
      std::vector<const Obstacle *> obstacles,
      const Mission::LaneChangeRequest lane_change_request,
      const CorridorInfo::Type type);

  /**
   * @brief assign lane boundary info to accoridor.
   *
   * @param local_route The local route data that contains information about the
   * current route.
   * @param corridor_point The corridor point whose lane boundary information
   * will be updated
   *
   */
  void AssignCorridorPointType(const LocalRoute &local_route,
                               CorridorPoint &corridor_point) const;

  /**
   * @brief Creat corridors infomation.
   *
   * @param current_local_route Current local route.
   * @param target_local_route Target local route.
   * @param corroidor_start_point Corridor start point.
   * @param v_target Target velocity.
   * @param corridor_type Corridor type.
   * @param trajectory Generate corridor trajectory.
   * @return CorridorInfo Corridor info created.
   */
  CorridorInfo CreateCorridorInfo(const LocalRoute &current_local_route,
                                  const LocalRoute &target_local_route,
                                  const CorridorPoint &corroidor_start_point,
                                  const double v_target,
                                  const CorridorInfo::Type corridor_type,
                                  const CartesianTrajectory &trajectory);

  /**
   * @brief add local route end as virtual obstacle in corridorinfo.
   *
   * @param corridor_info Corridor Info
   */
  void AddDestinationVirtualObstacle(CorridorInfo &corridor_info);

  /**
   * @brief check latest corridor valid.
   *
   */
  const bool CheckLatestCorridorValid();

  /**
   * @brief Check if nudge condition ready.
   * @param v_ego ego velocity
   * @param v_obstacle obstacle velocity
   * @return ready result
   *
   */
  bool CheckNudgeConditionReady(const double v_ego, const double v_obstacle);

  /**
   * @brief Check if the obstacle need ignore.
   * @param obstacle obstacle
   * @param ego_state ego status
   * @return ignore result
   *
   */
  bool CheckObstacleNeedIgnore(const Obstacle &obstacle,
                               const ::common::FrenetPoint &ego_state,
                               const bool is_lane_keep = false);
  /**
   * @brief Check last corridor info valid.
   *
   * @param frame_history Frame history
   * @return true
   * @return false
   */

  const bool CheckCorridorInfoPrevValid(
      const FrameHistory &frame_history) const;
  /**
   * @brief Check last corridor valid.
   *
   * @param frame_history Frame history
   * @return true
   * @return false
   */
  const bool CheckCorridorPrevValid(const FrameHistory &frame_history) const;

 private:
  static constexpr int kIdxL = 0;
  static constexpr int kIdxLDot = 1;
  static constexpr int kIdxLDDot = 2;
  static constexpr int kIdxS = 0;
  static constexpr int kIdxSDot = 1;
  static constexpr int kIdxSDDot = 2;

  LookupTable v_target_coeff_table_;
  LookupTable a_lat_max_table_;
  LateralDeciderConfig config_;
  double l_left_corr_ = 0.0;
  double l_right_corr_ = 0.0;

  std::unique_ptr<FrenentTrajectoryMapBuilder> frenet_trajectory_map_builder_;
  std::unique_ptr<CorridorInfoBuilder> corridor_info_builder_;

  FRIEND_TEST(LateralDeciderTest, TestComputeCorridorStartPoint);
  FRIEND_TEST(LateralDeciderTest, TestCreateCorridorInfo);
  FRIEND_TEST(LateralDeciderTest, TestComputeCorridor);
  FRIEND_TEST(LateralDeciderTest, TestComputeEndCorridorSegment);
  FRIEND_TEST(LateralDeciderTest, TestComputeMiddleCorridorSegment);
  FRIEND_TEST(LateralDeciderTest, TestCalcLLeftAndRightBasedOnCurb);
  FRIEND_TEST(LateralDeciderTest, TestFilterObstacles);
  FRIEND_TEST(LateralDeciderTest, TestLCFilterObstacles);
  FRIEND_TEST(LateralDeciderTest, TestEmptyLCFilterObstacles);
  FRIEND_TEST(LateralDeciderTest, TestAssignCorridorPointType);
};

}  // namespace planning
}  // namespace zark
