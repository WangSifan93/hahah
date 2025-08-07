/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file road_section_info.h
 **/

#pragma once

#include <list>

#include "apps/planning/src/config/conf_gflags.h"
#include "messages/map_service/all_map_new.pb.h"
#include "messages/planning/ads_decision.pb.h"

namespace zark {
namespace planning {

const uint32_t kMaxSectionSize = 50;
const uint8_t kMaxLaneSize = 10;
const float kValidMaxSpeed = 140;  // [kph]
const float kSpeedMpsToKph = 3.6;
const float kMaxCurvDistance = 200.0;  //[m]

/**
 * @struct LaneTopoInfo
 * @brief Structure to store topological and navigation change relationship
 * information about a lane.
 */
struct LaneTopoInfo {
  LaneTopoInfo()
      : lane_topo(),
        change_direction(
            zark::ads_common::eLCRD_ForceLaneChgReqDir::LCRD_None_LC),
        is_recommend(true),
        forward_is_recommend(true),
        remain_distance(DecisionGflags::alc_forcelc_distlim_valid),
        non_recommend_distance(DecisionGflags::alc_forcelc_distlim_valid),
        lane_sequence(0),
        change_count(0),
        speed_limit(0.0),
        final_id("connect_dest"),
        be_merged(false),
        dist_merge_end(DecisionGflags::alc_forcelc_distlim_valid),
        merge_dir(zark::ads_common::eLCRD_ForceLaneChgReqDir::LCRD_None_LC),
        merged_id("continue"),
        be_split(false),
        dist_split_end(DecisionGflags::alc_forcelc_distlim_valid),
        split_dir(zark::ads_common::eLCRD_ForceLaneChgReqDir::LCRD_None_LC),
        split_id("continue"),
        remain_check_boundary(false) {}

  void reset() {
    change_direction = zark::ads_common::eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
    is_recommend = true;
    forward_is_recommend = true;
    remain_distance = DecisionGflags::alc_forcelc_distlim_valid;
    non_recommend_distance = DecisionGflags::alc_forcelc_distlim_valid;
    change_count = 0;
    final_id = "connect_dest";
    be_merged = false;
    dist_merge_end = DecisionGflags::alc_forcelc_distlim_valid;
    merge_dir = zark::ads_common::eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
    merged_id = "continue";
    be_split = false;
    dist_split_end = DecisionGflags::alc_forcelc_distlim_valid;
    split_dir = zark::ads_common::eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
    split_id = "continue";
    remain_check_boundary = false;
  }

  zark::hdmap_new::LaneInfo lane_topo;  // map topo info of this lane
  zark::ads_common::eLCRD_ForceLaneChgReqDir
      change_direction;  // direction of this lane change to ramp or main road
  bool is_recommend;     // this lane is a recommended lane
  bool forward_is_recommend;  // this lane's forward connected lane is a
                              // recommended lane
  float remain_distance;  // the max length of this lane can connected in route
                          // path
  float non_recommend_distance;  // the distance between this lane to it's
                                 // connected non_recommend lane
  uint8_t lane_sequence;  // sequence result from right to left in same segment
  uint8_t change_count;   // count of this lane change to ramp or main road
  float speed_limit;      // [kph]
  std::string final_id;   // not recommended lane, final lane id;
  bool be_merged;
  float dist_merge_end;
  zark::ads_common::eLCRD_ForceLaneChgReqDir merge_dir;
  std::string merged_id;
  bool be_split;
  float dist_split_end;
  zark::ads_common::eLCRD_ForceLaneChgReqDir split_dir;
  std::string split_id;
  bool remain_check_boundary;  // check lane solid boundary;
};

/**
 * @struct SectionTopoRange
 * @brief Structure to store range of back topological information about a
 * road section.
 */
struct SectionTopoRange {
  SectionTopoRange()
      : priority_sucessor_range(),
        first_successor_seq(kMaxLaneSize),
        last_successor_seq(0) {}

  std::map<uint8_t, std::string>
      priority_sucessor_range;  // key is current lane seq, value is forward
                                // connect lane id;
  uint8_t first_successor_seq;
  uint8_t last_successor_seq;
};

/**
 * @struct NavSection
 * @brief Structure to store information about a road section.
 */
struct NavSection {
  NavSection()
      : lanes(),
        topo_range(),
        start_s(DecisionGflags::alc_forcelc_distlim_valid),
        end_s(DecisionGflags::alc_forcelc_distlim_valid),
        valid_lane_size(0),
        section_id(),
        all_lane_connect(true),
        all_transition_continue(true),
        next_new_road(false),
        min_spee_limit(kValidMaxSpeed),
        max_speed_limit(0.0),
        attribute() {}

  std::unordered_map<std::string, LaneTopoInfo>
      lanes;  // map of LaneTopoInfo structures, key is lane sequence;
  SectionTopoRange topo_range;  // Topological range information
  float start_s;                // Distance between ego and segment start
  float end_s;                  // Distance between ego and segment end
  uint8_t valid_lane_size;      // Actual number of lanes included
  std::string section_id;       // Unique id for every segment
  bool all_lane_connect;        // Indicates if all lanes are connected
  bool all_transition_continue;
  bool next_new_road;     // Indicates if the next road is different from the
                          // current road
  float min_spee_limit;   // [kph]
  float max_speed_limit;  // [kph]
  int32_t attribute;
};

struct NoaLine {
  NoaLine()
      : is_trigger(false), direction(0), last_trigger_time(0.0), trigger_id() {}

  void ResetTriggerInfo() {
    is_trigger = false;
    direction = 0;
    last_trigger_time = 0.0;
    trigger_id.clear();
  }

  bool is_trigger;
  int8_t direction;  // -1 right; 0 non direction; 1 left
  double last_trigger_time;
  std::string trigger_id;
};

struct SpecialSection {
 public:
  SpecialSection()
      : distance_section(DecisionGflags::alc_forcelc_distlim_valid),
        id(),
        target_section_id(),
        attribute(0),
        valid(false) {}

  float distance_section;
  std::string id;
  std::string target_section_id;
  int32_t attribute;
  bool valid;
};

struct NavigationInfo {
 public:
  NavigationInfo()
      : sections(),
        error_sections(),
        change_points(),
        noa_line(),
        current_lane_info(),
        current_section_id(),
        currrent_lane_pose(0),
        current_lane_id(),
        ego_speed(0.0),
        valid_info(false),
        special_section(),
        special_section_ids(),
        info_timestamp(0.0) {}

 public:
  std::list<NavSection> sections;
  std::list<NavSection> error_sections;
  std::vector<zark::hdmap_new::ChangePoint> change_points;
  NoaLine noa_line;
  LaneTopoInfo current_lane_info;
  std::string current_section_id;
  uint8_t currrent_lane_pose;  // sequence from right(right most set as 1) to
                               // left in section
  std::string current_lane_id;
  float ego_speed;  // [m/s]
  bool valid_info;
  SpecialSection special_section;
  std::unordered_set<std::string> special_section_ids;
  double info_timestamp;
};

struct FixNoaLineInfo {
  FixNoaLineInfo()
      : t(0.0),
        fix_vel(0.0),
        final_s(0.0),
        start_heading(0.0),
        final_heading(0.0),
        start_lane_id(),
        final_lane_id(),
        is_continue_split(false),
        valid(false) {}

  void ClearFixInfo() {
    t = 0.0;
    fix_vel = 0.0;
    final_s = 0.0;
    start_heading = 0.0;
    final_heading = 0.0;
    start_lane_id.clear();
    final_lane_id.clear();
    is_continue_split = false;
    valid = false;
  }

  double t;              // The time variable connecting the starting point and
                         // ending point of a fifth degree polynomial
  double fix_vel;        // intial and end state of velocity
  double final_s;        // end point frenet s at end lane;
  double start_heading;  // start point heading
  double final_heading;  // end point heading
  std::string start_lane_id;  // lane id of start point
  std::string final_lane_id;  // lane id of end point
  bool is_continue_split;
  bool valid;
};

struct BoundaryMoveInfo {
  BoundaryMoveInfo() : move_base(0), start_timestamp(0.0) {}
  void ResetMoveInfo() {
    move_base = 0;
    start_timestamp = 0.0;
  }

  int8_t move_base;  // 0: center; 1: base left; -1: base right
  double start_timestamp;
};

}  // namespace planning
}  // namespace zark
