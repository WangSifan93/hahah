//
// Copyright 2024 ZDrive.AI. All Rights Reserved.
//

/**
 * @file
 **/

#pragma once

#include <limits>
#include <string>
#include <vector>

#include "pnc_point.h"
#include "apps/planning/src/common/vehicle_state/proto/vehicle_state.h"
#include "apps/planning/src/reference_line_provider/pnc_map/path.h"
#include "messages/mapfusion/routing.pb.h"

namespace zark {
namespace planning {
namespace hdmap {

/**
 * @brief class RouteSegments
 *
 * This class is a representation of the Passage type in routing.proto.
 * It is extended from a passage region, but keeps some properties of the
 * passage, such as the last end LaneWaypoint of the original passage region
 * (route_end_waypoint), whether the passage can lead to another passage in
 * routing (can_exit_).
 * This class contains the original data that can be used to generate
 * hdmap::Path.
 **/
class RouteSegments : public std::vector<LaneSegment> {
 public:
  /**
   * The default constructor.
   **/
  RouteSegments() = default;

  /**
   * Get the next change lane action need to take by the vehicle, if the vehicle
   * is on this RouteSegments.
   * --- If the vehicle does not need to change lane, then change_lane_type ==
   *     routing::FORWARD;
   * --- If the vehicle need to change to left lane according to routing, then
   *     change_lane_type_ == routing::LEFT;
   * --- If the vehicle need to change to right lane according to routing, then
   *     change_lane_type_ == routing::RIGHT;
   */
  zark::routing::ChangeLaneType NextAction() const;
  void SetNextAction(zark::routing::ChangeLaneType action);

  /**
   * Get the previous change lane action need to take by the vehicle to reach
   * current segment, if the vehicle is not on this RouteSegments.
   * If the vehicle is already on this segment, or does not need to change lane
   * to reach this segment, then change_lane_type = routing::FORWARD;
   * If the vehicle need to change to left to reach this segment, then
   * change_lane_type_ =  routing::LEFT;
   * If the vehicle need to change to right to reach this segment, then
   * change_lane_type_ = routing::RIGHT;
   */
  zark::routing::ChangeLaneType PreviousAction() const;
  void SetPreviousAction(zark::routing::ChangeLaneType action);

  /**
   * Whether the passage region that generate this route segment can lead to
   * another passage region in route.
   */
  bool CanExit() const;
  void SetCanExit(bool can_exit);

  /**
   * Project a point to this route segment.
   * @param point_enu a map point, or point, which is a Vec2d point
   * @param s return the longitudinal s relative to the route segment.
   * @param l return the lateral distance relative to the route segment.
   * @param waypoint return the LaneWaypoint, which has lane and lane_s on the
   * route segment.
   * @return false if error happened or projected outside of the lane segments.
   */
  bool GetProjection(const ::common::PointENU &point_enu,
                     ::common::SLPoint *sl_point, LaneWaypoint *waypoint) const;
  bool GetProjection(const ::math::Vec2d &point, ::common::SLPoint *sl_point,
                     LaneWaypoint *waypoint) const;

  bool GetWaypoint(const double s, LaneWaypoint *waypoint) const;

  /**
   * @brief Check whether the map allows a vehicle can reach current
   * RouteSegment from a point on a lane (LaneWaypoint).
   * @param waypoint the start waypoint
   * @return true if the map allows a vehicle to drive from waypoint to
   * current RouteSegment. Otherwise false.
   */
  bool CanDriveFrom(const LaneWaypoint &waypoint,
                    const bool &is_online_map) const;

  /*
   * This is the point that is the end of the original passage in routing.
   * It is used to check if the vehicle is out of current routing.
   * The LaneWaypoint.lane is nullptr if the end of the passage is not on the
   * RouteSegment.
   */
  const LaneWaypoint &RouteEndWaypoint() const;
  void SetRouteEndWaypoint(const LaneWaypoint &waypoint);

  /** Stitch current route segments with the other route segment.
   * Example 1
   * this:   |--------A-----x-----B------|
   * other:                 |-----B------x--------C-------|
   * Result: |--------A-----x-----B------x--------C-------|
   * In the above example, A-B is current route segments, and B-C is the other
   * route segments. We update current route segments to A-B-C.
   *
   * Example 2
   * this:                  |-----A------x--------B-------|
   * other:  |--------C-----x-----A------|
   * Result: |--------C-----x-----A------x--------B-------|
   * In the above example, A-B is current route segments, and C-A is the other
   * route segments. We update current route segments to C-A-B
   *
   * @return false if these two reference line cannot be stitched
   */
  bool Stitch(const RouteSegments &other);

  bool Shrink(const ::math::Vec2d &point, const double look_backward,
              const double look_forward);

  bool Shrink(const double s, const double look_backward,
              const double look_forward);

  bool Shrink(const double s, const LaneWaypoint &waypoint,
              const double look_backward, const double look_forward);

  bool IsOnSegment() const;
  void SetIsOnSegment(bool on_segment);

  bool IsNeighborSegment() const;
  void SetIsNeighborSegment(bool is_neighbor);

  void SetId(const std::string &id);
  const std::string &Id() const;

  /**
   * Get the first waypoint from the lane segments.
   */
  LaneWaypoint FirstWaypoint() const;

  /**
   * Get the last waypoint from the lane segments.
   */
  LaneWaypoint LastWaypoint() const;

  /**
   * @brief Check if a waypoint is on segment
   */
  bool IsWaypointOnSegment(const LaneWaypoint &waypoint) const;

  /**
   * @brief Check if we can reach the other segment from current segment just
   * by following lane.
   * @param other Another route segment
   */
  bool IsConnectedSegment(const RouteSegments &other) const;

  bool StopForDestination() const;
  void SetStopForDestination(bool stop_for_destination);

  /**
   * @brief Copy the properties of other segments to current one
   */
  void SetProperties(const RouteSegments &other);

  static bool WithinLaneSegment(const LaneSegment &lane_segment,
                                const LaneWaypoint &waypoint);

  static bool WithinLaneSegment(const LaneSegment &lane_segment,
                                const zark::routing::LaneWaypoint &waypoint);

  static bool WithinLaneSegment(const zark::routing::LaneSegment &lane_segment,
                                const LaneWaypoint &waypoint);

  static bool WithinLaneSegment(const zark::routing::LaneSegment &lane_segment,
                                const zark::routing::LaneWaypoint &waypoint);

  static double Length(const RouteSegments &segments);

  bool isCloseNextPassage() const;  // change lane use to decrease cost.

  void setNeighborNum(const std::pair<int, int> &num) { neibor_num_ = num; }

  const std::pair<int, int> getNeighborNum() const { return neibor_num_; }

  void SetDiffRoadDis(const double &distance) { diff_road_dis_ = distance; }
  const double GetDiffRoadDis() const { return diff_road_dis_; }

  void SetSepratDirection(const int &dir) { next_separat_direction_ = dir; }
  const int GetSepratDirection() const { return next_separat_direction_; }

  void SetRampConnectId(const std::vector<std::string> &connect_ids);
  const std::vector<std::string> GetRampConnectId() const {
    return ramp_connect_ids_;
  }

  enum class SegmentType : uint32_t {
    InvalidSegment = 0,
    CurrentSegment = 1,
    LeftSegment = 10,
    RightSegment = 100,
    MapCenterLine = 1000,
  };
  const SegmentType GetSegmentType() const { return segment_type_; }
  void SetSegmentType(const SegmentType &type) { segment_type_ = type; }

 private:
  SegmentType segment_type_ = SegmentType::InvalidSegment;
  bool ego_left_segment_ = false;
  bool ego_right_segmetn_ = false;
  LaneWaypoint route_end_waypoint_;

  /**
   * whether this segment can lead to another passage region in routing
   */
  bool can_exit_ = false;

  /**
   * Indicates whether the vehicle is on current RouteSegment.
   **/
  bool is_on_segment_ = false;

  /**
   * Indicates whether current routeSegment is the neighbor of vehicle
   * routeSegment.
   **/
  bool is_neighbor_ = false;

  zark::routing::ChangeLaneType next_action_ = zark::routing::FORWARD;

  zark::routing::ChangeLaneType previous_action_ = zark::routing::FORWARD;

  std::string id_;

  /**
   * Whether the vehicle should stop for destination. In a routing that has
   * loops, the adc may pass by destination many times, but it only need to stop
   * for destination  in the last loop.
   */
  bool stop_for_destination_ = false;

  // 当前路段的左/右通道数；
  std::pair<int, int> neibor_num_ = std::make_pair(0, 0);

  // distance between current pose to next Separation；
  // next separation direction: 0 invalid, 1 left, 2 right
  double diff_road_dis_ = std::numeric_limits<double>::max();
  int next_separat_direction_ = 0;
  std::vector<std::string> ramp_connect_ids_;
};

}  // namespace hdmap
}  // namespace planning
}  // namespace zark
