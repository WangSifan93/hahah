/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
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
 * @file local_route.h
 **/

#pragma once

#include <vector>

#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/common/corridor.h"
#include "apps/planning/src/common/local_route/local_route_point.h"
#include "apps/planning/src/common/obstacle.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/planning_msgs/local_route_config.h"
#include "apps/planning/src/planning_msgs/sl_boundary.h"
#include "apps/planning/src/reference_line_provider/pnc_map/hdmap_api.h"
#include "apps/planning/src/reference_line_provider/pnc_map/route_segments.h"
#include "linear_interpolation.h"

namespace zark {
namespace planning {

constexpr double kMaxForwardDistacne = 5000.0;  // [m]

class LocalRoute {
 public:
  LocalRoute() = default;

  LocalRoute(const LocalRoute& local_route) = default;

  explicit LocalRoute(const LocalRouteConfig& config);

  explicit LocalRoute(const std::vector<LocalRoutePoint>& local_route_points,
                      const LocalRouteConfig& config);

  explicit LocalRoute(const std::vector<LocalRoutePoint>& local_route_points,
                      const LocalRouteConfig& config, const bool is_side_line);

  explicit LocalRoute(const hdmap::RouteSegments& segments,
                      const LocalRouteConfig& config);

  enum class RouteDirection { CENTER = 0, LEFT = 1, RIGHT = 2 };

  /**
   * @struct RouteLaneChangeInfo
   * @brief contains route lane change info of this lane for planning
   */
  struct RouteLaneChangeInfo {
    // lane change directions from the current lane to the next and second next
    // connecting lane
    std::vector<RouteDirection> dir_next_lc = {RouteDirection::CENTER,
                                               RouteDirection::CENTER};
    // distances to the next and second next lane change point or intersection
    // [m]
    std::vector<double> dist_to_next_lc_pt = {kMaxForwardDistacne,
                                              kMaxForwardDistacne};
    // number of lane changes from the current lane to the next and second next
    // connecting lanes [-]
    std::vector<int> num_lc_to_next_lanes = {0, 0};
  };

  /**
   * @struct LineBoundary
   * @brief contains left and right boundary type between start_s to end_s
   */
  struct LineBoundary {
    LineBoundary() = default;
    LineBoundary(const hdmap_new::LaneBoundaryAttribute::Type& l_type,
                 const hdmap_new::LaneBoundaryAttribute::Type& r_type,
                 const double s_start, const double s_end)
        : left_type(l_type),
          right_type(r_type),
          start_s(s_start),
          end_s(s_end) {}

    hdmap_new::LaneBoundaryAttribute::Type left_type =
        hdmap_new::LaneBoundaryAttribute::LANEBOUNDARY_TYPE_UNKNOWN;
    hdmap_new::LaneBoundaryAttribute::Type right_type =
        hdmap_new::LaneBoundaryAttribute::LANEBOUNDARY_TYPE_UNKNOWN;
    double start_s =
        0.0;             // boundary_type start distance in the referen line [m]
    double end_s = 0.0;  // boundary_type end distance in the referen line [m]
  };

  /**
   * @brief init local_route with environment info
   * @param obstacles all obstacle from upstream
   * @param vehicle_state ego pose info
   * @param planning_start_point planning start point
   * @return return true if init succeed
   */
  bool Init(const common::VehicleState& vehicle_state,
            const ::common::TrajectoryPoint& planning_start_point);

  /**
   * @brief get route lane change information of this local route for
   * planning
   */
  inline const RouteLaneChangeInfo GetRouteLCInfo() const {
    return route_lc_info_;
  }

  /**
   * @brief get reference points
   */
  inline const std::vector<LocalRoutePoint> RoutePoints() const {
    return route_points_;
  }

  /**
   * @brief check whether ego is on this local route
   * @return return true if ego is on this local route
   */
  inline const bool IsEgoCurrentRoute() const { return on_current_route_; }

  /**
   * @brief get the state of ego in the frenet coordinate
   */
  inline const FrenetPoint InitFrenetPoint() const {
    return init_frenet_point_;
  }

  inline void SetInitFrenetState(const FrenetPoint& init_frenet_point) {
    init_frenet_point_ = init_frenet_point;
  }

  /**
   * @brief get all lanes contained by this local route
   */
  inline const hdmap::RouteSegments& Lanes() const { return lanes_; }

  inline void SetLanes(const hdmap::RouteSegments& lanes) { lanes_ = lanes; }

  /**
   * @brief map_path_info is obtained by mapping the points of this reference
   * line to center line of hdmap. get map_path_info for more hdmap info in
   * every local_rote_point.
   */
  inline const hdmap::Path& MapPathInfo() const { return map_path_info_; }

  /**
   * @brief get left local route pointers of the current local route
   */
  inline LocalRoute* LeftRoute() const { return left_route_; }

  /**
   * @brief get right local route pointers of the current local route
   */
  inline LocalRoute* RightRoute() const { return right_route_; }

  /**
   * @brief Set the Left LocalRoute object
   *
   * @param left_route
   */
  inline void SetLeftRoute(LocalRoute* left_route) { left_route_ = left_route; }

  /**
   * @brief Set the Right LocalRoute object
   *
   * @param right_route
   */
  inline void SetRightRoute(LocalRoute* right_route) {
    right_route_ = right_route;
  }

  /**
   * @brief get width from left and right boundary to centerline at distance s
   * of this lane
   */
  bool GetLaneWidth(const double s, double& lane_left_width,
                    double& lane_right_width) const;

  /**
   * @brief get width from left and right boundary to centerline at distance s
   * of this road
   */
  bool GetRoadWidth(const double s, double& road_left_width,
                    double& road_right_width) const;

  /**
   * @brief get width from left and right curb to centerline at distance s
   * of this road
   */
  bool GetDistanceToCurb(const double s, double& road_left_width,
                         double& road_right_width) const;

  /**
   * @brief get left and right line Boundary between current s to distance
   */
  const std::vector<LineBoundary> GetLaneBoundary(const double s,
                                                  const double distance) const;

  /**
   * @brief get map lane speed limit at distance s of this local route
   */
  const double GetSpeedLimitFromS(const double s) const;

  /**
   * @brief get map road class at distance s of this local route
   */
  const hdmap_new::Road_RoadClass GetRoadClassFromS(const double s) const;

  /**
   * @brief get map road boundary type at distance s of this local route
   */
  void GetRoadBoundaryTypeFromS(
      const double s,
      hdmap_new::RoadBoundary::RoadBoundaryType::Type& left_boundary_type,
      hdmap_new::RoadBoundary::RoadBoundaryType::Type& right_boundary_type)
      const;

  /**
   * @brief get map lane type at distance s of this local route
   */
  const hdmap_new::Lane_LaneType GetLaneTypeFromS(const double s) const;

  /**
   * @brief get RoutePoint at distance s of this local route
   */
  const LocalRoutePoint GetLocalRoutePoint(const double s) const;

  std::vector<LocalRoutePoint> GetLocalRoutePoints(double start_s,
                                                   double end_s) const;

  /**
   * @brief get Curvature at distance s of this local route
   */
  const double GetCurvatureFromS(const double s) const;

  const double GetDrivingWidth(const SLBoundary& sl_boundary) const;

  bool IsOnLane(const SLBoundary& sl_boundary) const;

  inline const double Length() const { return map_path_info_.length(); }

  inline bool IsChangeLanePath() const { return !Lanes().IsOnSegment(); }

  bool XYToSL(const ::math::Vec2d& xy_point, ::common::SLPoint& sl_point) const;

  template <class XYPoint>
  bool XYToSL(const XYPoint& xy, ::common::SLPoint& sl_point) const {
    return XYToSL(::math::Vec2d(xy.x(), xy.y()), sl_point);
  }

  bool SLToXY(const ::common::SLPoint& sl_point, ::math::Vec2d& xy_point) const;

  bool SLToXY(const ::common::SLPoint& sl_point, ::math::Vec2d& xy_point,
              LocalRoutePoint& matched_point) const;

  FrenetPoint ToFrenetFrame(const TrajectoryPoint& traj_point) const;

  FrenetPoint ToFrenetFrame(const CorridorPoint& traj_point) const;

  /**
   * @brief get route points index which is Nearest the given s
   */
  const size_t GetNearestPointIndex(const double s) const;

  /**
   * @brief shrink this local route line within given s between backward
   * distance to forward distance
   */
  bool Segment(const double s, const double look_backward,
               const double look_forward);

  /** Stitch current local route line with the other local route line
   * The stitching strategy is to use current local route points as much as
   * possible. The following two examples show two successful stitch cases.
   *
   * Example 1
   * this:   |--------A-----x-----B------|
   * other:                 |-----C------x--------D-------|
   * Result: |------A-----x-----B------x--------D-------|
   * In the above example, A-B is current local route line, and C-D is the other
   * local route line. If part B and part C matches, we update current local
   * route line to A-B-D.
   *
   * Example 2
   * this:                  |-----A------x--------B-------|
   * other:  |--------C-----x-----D------|
   * Result: |--------C-----x-----A------x--------B-------|
   * In the above example, A-B is current local route line, and C-D is the other
   * local route line. If part A and part D matches, we update current local
   * route line to C-A-B.
   *
   * @return false if these two local route line cannot be stitched
   */
  bool Stitch(const LocalRoute& other);

  /**
   * @brief get route points which is Nearest the given s
   */
  LocalRoutePoint GetNearestLocalRoutePoint(const double s) const;

  /**
   * @brief Check if current route is the target route when has lane change
   * deicision
   */
  inline const bool IsTargetRoute() const { return is_target_route_; }

  inline void SetIsTargetRoute(const bool is_target_route) {
    is_target_route_ = is_target_route;
  }

  bool AddObstaclesSLBoundary(const std::vector<const Obstacle*>& obstacles);

  bool AddObstacleSLBoundary(const Obstacle* obstacle);

  bool GetSLBoundary(const ::math::Box2d& box, SLBoundary& sl_boundary) const;

  const double GetEndS() const;

  inline void SetDestinationS(const double s) { s_destination_ = s; }

  /**
   * @brief get the distance to destination
   */
  const inline double GetDestinationS() const { return s_destination_; }

  inline void SetConfig(const LocalRouteConfig& config) { config_ = config; }

  const inline LocalRouteConfig GetConfig() const { return config_; }

  inline void SetHostLC2Left(const bool result) { host_lc_to_left_ = result; }
  inline void SetHostLC2Right(const bool result) { host_lc_to_right_ = result; }

  /**
   * @brief get the flag of current local route change to left
   */
  const inline bool IsHostLC2Left() const { return host_lc_to_left_; }

  /**
   * @brief get the flag of current local route change to right
   */
  const inline bool IsHostLC2Right() const { return host_lc_to_right_; }

 private:
  const LocalRoutePoint InterpolateWithMatchedIndex(
      const LocalRoutePoint& p0, const double s0, const LocalRoutePoint& p1,
      const double s1, const hdmap::InterpolatedIndex& index) const;

  bool IsIrrelevantObstacle(const SLBoundary& adc_sl_boundary,
                            const Obstacle& obstacle);

  const std::vector<const Obstacle*> FilterObstacles(
      const std::vector<const Obstacle*>& obstacles,
      const SLBoundary& adc_sl_boundary);

 private:
  bool on_current_route_;
  bool is_target_route_;
  double s_destination_;
  LocalRoute* left_route_;
  LocalRoute* right_route_;
  std::vector<LocalRoutePoint> route_points_;
  hdmap::Path map_path_info_;
  hdmap::RouteSegments lanes_;
  FrenetPoint init_frenet_point_;
  RouteLaneChangeInfo route_lc_info_;
  IndexedList<const Obstacle*, SLBoundary> obstacle_sl_boundaries_;
  LocalRouteConfig config_;
  bool host_lc_to_left_{false};
  bool host_lc_to_right_{false};
};

}  // namespace planning
}  // namespace zark
