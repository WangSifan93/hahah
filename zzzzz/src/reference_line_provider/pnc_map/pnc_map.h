//
// Copyright 2024 ZDrive.AI. All Rights Reserved.
//

/**
 * @file:
 **/

#pragma once

#include <list>
#include <string>
#include <unordered_set>
#include <vector>

#include "apps/planning/src/common/vehicle_state/proto/vehicle_state.h"
#include "apps/planning/src/common/data_reader/road_section_info.h"
#include "apps/planning/src/common/data_reader/data_reader.h"
#include "gtest/gtest_prod.h"
#include "apps/planning/src/planning_msgs/local_route_config.h"
#include "apps/planning/src/reference_line_provider/pnc_map/path.h"
#include "apps/planning/src/reference_line_provider/pnc_map/hdmap_api.h"
#include "apps/planning/src/reference_line_provider/pnc_map/route_segments.h"

namespace zark {
namespace planning {
namespace hdmap {

const double kPointDistBuffer = 300.0;
const double kLimitDisBuff = 10.0;
const double kSectionDisBuffer = 100.0;
const double kFollowRampSectionDis = 100.0;
const double kDistThreshold = 5.0;  // [m]
const double kMinRemainDist = 1.0;  // [m]
const double kDistExceedRatio = 0.05;
const int8_t kMinPointSize = 2;

class PncMap {
 public:
  virtual ~PncMap() = default;
  explicit PncMap();

  bool UpdateLocalHdMap(const MapInfo &new_local_map,
                        const bool &is_online_map);

  void init();

  void ClearNavInfoGrouop();

  bool UpdateRoutingResponse(const zark::hdmap_new::Path &main_path);

  bool updateRouting(const zark::hdmap_new::Path &routing_path);

  const routing::RoutingResponse &routing_response() const;

  double LookForwardDistance(const double velocity);

  bool GetRouteSegments(const common::VehicleState &vehicle_state,
                        const double backward_length,
                        const double forward_length,
                        std::list<RouteSegments> *const route_segments,
                        const bool &is_online_map);
  /**
   * @brief use heuristic forward length and backward length
   */
  bool GetRouteSegments(const common::VehicleState &vehicle_state,
                        std::list<RouteSegments> *const route_segments,
                        const bool &is_online_map);

  /**
   * Check if the routing is the same as existing one in PncMap
   */
  bool IsNewRouting(
      const zark::routing::RoutingResponse &routing_response) const;
  static bool IsNewRouting(const routing::RoutingResponse &prev,
                           const routing::RoutingResponse &routing_response);

  bool ExtendSegments(const RouteSegments &segments,
                      const ::common::PointENU &point, double look_backward,
                      double look_forward, RouteSegments *extended_segments);

  bool ExtendSegments(const RouteSegments &segments, double start_s,
                      double end_s,
                      RouteSegments *const truncated_segments) const;

  std::vector<routing::LaneWaypoint> FutureRouteWaypoints() const;

  std::pair<int, int> setNeighborNum(zark::hdmap_new::Id current_lane_id);
  void SetLocalQuaternion(
      const std::pair<bool, zark::mapfusion::MapLocalizationInfo> q) {
    global2local_quaternion_ = q;
  }

  const bool UpdateMainPath() const { return update_main_path_; };

  const NavigationInfo *GetNavInfo() {
    std::lock_guard<std::mutex> lock(navinfo_lock_);
    NavigationInfo *nav_Info = nullptr;
    if (navigation_info_group_.size() == 0) {
      return nav_Info;
    } else {
      return &navigation_info_group_.back();
    }
  }

  struct Follow2RampInfo {
    Follow2RampInfo() : follow_to_ramp(false), section_id(), lane_id() {}

    bool follow_to_ramp;     // trigger noa line to ramp
    std::string section_id;  // section id
    std::string lane_id;     // need follow lane id
  };

  const LaneWaypoint &GetAdcWaypoint() const { return adc_waypoint_; }

  LaneInfoConstPtr GetRouteSuccessor(
      LaneInfoConstPtr lane, const RouteSegments::SegmentType &type) const;

  void SetFixNoaLineInfo(const FixNoaLineInfo &current_info) {
    last_noa_line_ = current_info;
  }

  const std::string GetErrorMsg() const { return error_msg_; }

  void ClearErrorMsg() { error_msg_.clear(); }

 private:
  bool UpdateVehicleState(const common::VehicleState &vehicle_state,
                          const bool &is_online_map);
  /**
   * @brief Find the waypoint index of a routing waypoint. It updates
   * adc_route_index_
   * @return index out of range if cannot find waypoint on routing, otherwise
   *   an index in range [0, route_indices.size());
   */
  int GetWaypointIndex(const LaneWaypoint &waypoint) const;

  bool GetNearestPointFromRouting(const common::VehicleState &point,
                                  LaneWaypoint *waypoint);

  bool PassageToSegments(const hdmap_new::Id &passage_lane_id,
                         RouteSegments *segments) const;

  bool ProjectToSegments(const ::common::PointENU &point_enu,
                         const RouteSegments &segments,
                         LaneWaypoint *waypoint) const;

  static bool ValidateRouting(const routing::RoutingResponse &routing);

  static void AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                 const double end_s,
                                 std::vector<MapPathPoint> *const points);

  LaneInfoConstPtr GetRoutePredecessor(
      LaneInfoConstPtr lane, const RouteSegments::SegmentType &type,
      const std::unordered_set<std::string> last_segment_ids) const;

  /**
   * Return the neighbor passages from passage with index start_passage on road.
   * @param road the road information from routing
   * @param start_passage the passsage index in road
   * @return all the indices of the neighboring passages, including
   * start_passage.
   */
  std::vector<std::pair<int, RouteSegments::SegmentType>> GetNeighborPassages(
      const zmap::RoadSectionInfoConstPtr &road_section,
      int start_passage) const;

  /**
   * @brief convert a routing waypoint to lane waypoint
   * @return empty LaneWaypoint if the lane id cannot be found on map, otherwise
   * return a valid LaneWaypoint with lane ptr and s.
   */
  LaneWaypoint ToLaneWaypoint(const routing::LaneWaypoint &waypoint) const;

  /**
   * @brief convert a routing segment to lane segment
   * @return empty LaneSegmetn if the lane id cannot be found on map, otherwise
   * return a valid LaneSegment with lane ptr, start_s and end_s
   */
  LaneSegment ToLaneSegment(const routing::LaneSegment &segment) const;

  /**
   * @brief Update routing waypoint index to the next waypoint that ADC need to
   * pass. The logic is by comparing the current waypoint's route index with
   * route_index and adc_waypoint_:
   * a. If the waypoint's route_index < route_index_, ADC must have passed
   * the waypoint.
   * b. If the waypoint's route_index == route_index_, ADC and the waypoint
   * is on the same lane, compare the lane_s.
   */
  void UpdateNextRoutingWaypointIndex(int cur_index);

  /**
   * @brief find the index of waypoint by looking forward from index start.
   * @return empty vector if not found, otherwise return a vector { road_index,
   * passage_index, lane_index}
   */
  int SearchForwardWaypointIndex(int start, const LaneWaypoint &waypoint) const;
  int SearchBackwardWaypointIndex(int start,
                                  const LaneWaypoint &waypoint) const;

  void UpdateRoutingRange(int adc_index);

  // calculate distance between current pose to next Separation
  // separation_direction: 0 invalid, 1 left, 2 right;
  // only contains param_dis_ramp_;
  bool CalculateDiffRoadDis(const common::VehicleState &vehicle_state,
                            const LaneWaypoint &segment_waypoint,
                            double &diff_dis, int &separation_direction,
                            int &left_change_num, int &right_change_num,
                            std::vector<std::string> &ramp_connect_ids);

  bool CheckLaneType(const int32_t &type);

  /**
   * @brief Extracts lane change information from the current lane to the
   * nearest change point lane.
   *
   * This function extracts lane change information from the current lane to the
   * nearest lane change point based on the provided guidance information.
   *
   * @param guidance_info The guidance information containing lane change points
   * and road change points details.
   * @param current_section_index The index of the current section in the
   * sections of guidance information.
   * @param sections_lane_queue A vector to store the lane change information
   * for each section.
   */
  void ExtractNavLanechangeInfo(const hdmap_new::Guidance &guidance_info,
                                const common::VehicleState &vehicle_state,
                                const std::string &current_section_id,
                                const std::string &current_lane_id,
                                NavigationInfo &navi_info);

  void SetLaneSequence(
      const hdmap_new::SectionInfo &map_section,
      const std::unordered_map<std::string, hdmap_new::LaneInfo> &lanes_topo,
      const std::string &current_lane_id, uint8_t &current_lane_seq,
      NavSection &section_lane_queue);

  void CalculateTopoRange(
      const NavSection &forward_section,
      const std::unordered_map<std::string, hdmap_new::LaneInfo> &lanes_topo,
      const std::string &ego_section_id, NavSection &current_section);

  bool CalculateChangeCount(
      const NavSection &forward_section,
      const std::unordered_map<std::string, uint32_t> speed_limits,
      const std::string &current_section_id, NavSection &current_section);

  bool IdentitySpecialSection(const NavSection &forward_section,
                              NavSection &current_section,
                              const hdmap_new::SectionInfo &map_section,
                              NavigationInfo &navi_info);

  void CalNextNonRecomDis(const NavSection &forward_section,
                          const std::string &next_lane_id,
                          LaneTopoInfo &current_lane);

  void UpdateCurrentSectionInfo(const hdmap_new::Guidance &map_guidance,
                                const common::VehicleState &vehicle_state,
                                const std::string &current_section_id,
                                const std::string &current_lane_id,
                                NavigationInfo &navi_info);

  double UpdateRemainDist(const NavSection &forward_section,
                          const NavSection &current_section,
                          const hdmap_new::SectionInfo &cur_map_section,
                          const uint8_t current_lane_seq,
                          const double distance_forward);

  double UpdateNonReconmmenDist(const NavSection &forward_section,
                                const NavSection &current_section,
                                const hdmap_new::SectionInfo &cur_map_section,
                                const uint8_t current_lane_seq,
                                const double distance_forward);

  void ResetNavInfo(NavigationInfo &navi_info);

  bool ForwardTriggerNoaLine(const std::list<NavSection> &sections,
                             const LaneTopoInfo &current_lane,
                             const common::VehicleState &vehicle_state,
                             NoaLine &last_trigger_info,
                             const bool &continue_check = false);

  bool CheckSectionTrigger(const NavSection &section,
                           const LaneTopoInfo &forward_lane,
                           const common::VehicleState &vehicle_state,
                           const double forward_trigger_distance,
                           NoaLine &last_trigger_info);

  bool FollowTrigger(const std::list<NavSection> &sections,
                     const LaneTopoInfo &current_lane,
                     const NoaLine &last_trigger_info,
                     const common::VehicleState &ego_state,
                     const std::string &final_id);

  bool KeepTrigger(const std::list<NavSection> &sections,
                   const LaneTopoInfo &current_lane, std::string final_id,
                   const double &time_now,
                   const common::VehicleState &ego_state, bool &is_keep,
                   NoaLine &last_trigger_info);

  void MergeCurLaneSelect(const common::VehicleState &state,
                          LaneWaypoint *check_lane);

  bool LonLaneSelect(const hdmap::LaneWaypoint &lane,
                     const common::VehicleState &ego_state);

  void ExtractNonTargetSplit(const NavSection &forward_section,
                             const std::string &ego_section_id,
                             std::list<NavSection> &sections,
                             NavSection &current_section);

  void FillFinalSplitInfo(const NavSection &current_section,
                          const LaneTopoInfo &current_lane,
                          const eLCRD_ForceLaneChgReqDir &split_dir,
                          std::list<NavSection> &sections);

  void CalSolidBoundaryRemainDist(const NavSection &forward_section,
                                  NavSection &check_section);

  void CheckBoundaryRemainDist(LaneTopoInfo &check_lane,
                               const bool &is_first_check = true);

  void CheckNeighborLaneRemain(NavSection &check_section,
                               LaneTopoInfo &check_lane);

  void ClearNavInfoGrouop(const std::string &error_msg);

  void SetErrorMsg(const std::string &msg) { error_msg_ = msg; }

  void CheckLastIdentifyLane(const LaneWaypoint &check_lane, double &distance);

 private:
  LocalRouteConfig local_route_config_;
  FixNoaLineInfo last_noa_line_;
  zark::hdmap_new::Path main_path_;
  zark::routing::RoutingResponse routing_;
  struct RouteIndex {
    LaneSegment segment;
    std::array<int, 3> index;
  };
  std::vector<RouteIndex> route_indices_;
  int range_start_ = 0;
  int range_end_ = 0;
  // routing ids in range
  std::unordered_set<std::string> range_lane_ids_;
  std::unordered_set<std::string> all_lane_ids_;

  /**
   * The routing request waypoints
   */
  struct WaypointIndex {
    LaneWaypoint waypoint;
    int index;
    WaypointIndex(const LaneWaypoint &waypoint, int index)
        : waypoint(waypoint), index(index) {}
  };

  // return the segment of an index
  int NextWaypointIndex(int index) const;

  std::vector<WaypointIndex> routing_waypoint_index_;
  /**
   * The next routing request waypoint index in routing_waypoint_index_
   */
  std::size_t next_routing_waypoint_index_ = 0;

  hdmap_new::Guidance guidance_info_;  // map guidance information

  NavigationInfo navigation_info_;

  std::unordered_set<std::string> virtual_section_ids_;

  std::list<NavigationInfo> navigation_info_group_;

  NoaLine last_trigger_info_;

  std::mutex navinfo_lock_;

  int64_t last_map_timestamp_ = -1;
  std::shared_ptr<zark::map_service::zmap::ZpilotMapInfo> new_hdmap_;
  bool is_same_routing_ = false;

  std::string error_msg_;

  /**
   * The state of the adc
   */
  common::VehicleState adc_state_;
  /**
   * A three element index: {road_index, passage_index, lane_index}
   */
  int adc_route_index_ = -1;
  /**
   * The waypoint of the autonomous driving car
   */
  LaneWaypoint adc_waypoint_;
  std::unordered_set<std::string> last_confirm_lanes_;

  /**
   * @brief Indicates whether the adc should start consider destination.
   * In a looped routing, the vehicle may need to pass by the destination
   * point
   * may times on the road, but only need to stop when it encounters
   * destination
   * for the last time.
   */
  bool stop_for_destination_ = false;

  bool update_main_path_ = false;

  std::mutex local_map_mutex_;

  zark::hdmap_new::MapHeader_MapType map_type_;

  // to do need param
  double param_dis_ramp_ = 2500.0;
  bool map_update_ = false;
  std::pair<bool, zark::mapfusion::MapLocalizationInfo>
      global2local_quaternion_;
  FRIEND_TEST(PncMapTest, UpdateRouting);
  FRIEND_TEST(PncMapTest, GetNearestPointFromRouting);
  FRIEND_TEST(PncMapTest, UpdateWaypointIndex);
  FRIEND_TEST(PncMapTest, UpdateNextRoutingWaypointIndex);
  FRIEND_TEST(PncMapTest, GetNeighborPassages);
  FRIEND_TEST(PncMapTest, NextWaypointIndex);
  FRIEND_TEST(PncMapTest, SearchForwardIndex_SearchBackwardIndex);
};

}  // namespace hdmap
}  // namespace planning
}  // namespace zark
