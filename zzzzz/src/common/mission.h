#pragma once

#include "apps/planning/src/common/local_route/local_route.h"

namespace zark {
namespace planning {

using Road_RoadClass = zark::hdmap_new::Road_RoadClass;

/**
 * @brief VehicleInfos:current vehicle speed and driver set cruise speed
 *
 */
struct VehicleInfos {
  double cur_adc_v;
  double driver_set_v;
};

/**
 * @brief CombineLaneInfo:use to contain lane id, local route, route segment
 *
 */
struct CombineLaneInfo {
  std::string id;
  LocalRoute local_route;
  hdmap::RouteSegments route_segment;
};

/**
 * @brief LaneInfos:current,left,right laneinfo, current lane width, current
 * road type,current vehicle sl,routing lane change infos etc.
 *
 */
struct LaneInfos {
  std::string closer_lane_id;
  CombineLaneInfo current_laneinfo;
  CombineLaneInfo left_laneinfo;
  CombineLaneInfo right_laneinfo;
  std::vector<std::string> connect_lanes_id;
  double current_lane_width;
  Road_RoadClass cur_road_type;
  SLPoint current_sl;
  LocalRoute::RouteLaneChangeInfo route_lc_info;
};

/**
 * @brief ObstacleInfos:contain current,left,right,lleft,rright lane's obstacle
 *
 */
struct ObstacleInfos {
  std::vector<const Obstacle*> cur_lane_obs;
  std::vector<const Obstacle*> left_lane_obs;
  std::vector<const Obstacle*> right_lane_obs;
  std::vector<const Obstacle*> lleft_lane_obs;
  std::vector<const Obstacle*> rright_lane_obs;
};

/**
 * @brief CurLaneBlockInfos: flag is current lane blocked and distance to block
 * obstacle
 *
 */
struct CurLaneBlockInfos {
  bool is_current_line_blocked;
  double dis_2_frontstaticobj;
};

/**
 * @brief LaneSpdInfos:all lane's pass speed and limit speed
 *
 */
struct LaneSpdInfos {
  double v_cur_lane_limit;
  double v_left_lane_limit;
  double v_right_lane_limit;
  double v_cur_lane_pass;
  double v_left_lane_pass;
  double v_right_lane_pass;
};

/**
 * @brief RCParamInfos: routing lc params
 *
 */
struct RCParamInfos {
  double dis_2_ramp_threshold;
  double reverse_dis_2_ramp_threshold;
};

struct InhibitInfos {
  enum InhibitType {
    NONE = 0,
    CUR_LANE_INVALID = 1,
    LANE_TYPE_INVALID = 2,
    CURVATURE_INVALID = 3,
    LEFT_LANE_INVALID = 4,
    RIGHT_LANE_INVALID = 5,
  };

  bool is_inhibit_left;
  InhibitType inhibit_left_type;
  bool is_inhibit_right;
  InhibitType inhibit_right_type;
};

/**
 * @brief Hazard Vehicle Info
 *
 */
struct HazardVehInfo {
  bool is_cur_has_hazard;
  bool is_left_has_hazard;
  bool is_right_has_hazard;

  void Reset() {
    this->is_cur_has_hazard = false;
    this->is_left_has_hazard = false;
    this->is_right_has_hazard = false;
  }
};

/**
 * @brief Efficient LC Lane Speed Infos
 *
 */
struct EfficentLaneInfo {
  bool efficent_lane_left;
  bool efficent_lane_right;

  void Reset() {
    this->efficent_lane_left = false;
    this->efficent_lane_right = false;
  }
};

/**
 * @brief EnvInfos:all env infos
 *
 */
struct EnvInfos {
  double time_now;
  VehicleInfos vehicle_infos;
  LaneInfos lane_infos;
  ObstacleInfos obstacle_infos;
  CurLaneBlockInfos cur_lane_block_infos;
  LaneSpdInfos lane_spd_infos;
  double fury_value;
  RCParamInfos rc_params_infos;
  InhibitInfos inhibit_infos;
};

struct LCFlags {
  bool is_need_lc;
  bool is_lc_waiting;

  uint8_t right_turnlight;
  uint8_t left_turnlight;

  double time_lc_triggered;
  double time_lc_req_finish;
  double time_lc_req_fail;
  double time_cur_hazard_start;
  double time_left_hazard_clear_start;
  double time_right_hazard_clear_start;
  double time_cur_low_speed_start;
  double time_efficent_lane_triggered;

  void Reset() {
    this->is_need_lc = false;
    this->is_lc_waiting = false;

    this->right_turnlight = 0;
    this->left_turnlight = 0;

    this->time_lc_triggered = 0.0;
    this->time_lc_req_finish = 0.0;
    this->time_lc_req_fail = 0.0;
    this->time_cur_hazard_start = 0.0;
    this->time_left_hazard_clear_start = 0.0;
    this->time_right_hazard_clear_start = 0.0;
    this->time_cur_low_speed_start = 0.0;
    this->time_efficent_lane_triggered = 0.0;
  }
};

struct Mission {
  enum LaneChangeRequest { LANE_KEEP = 0, LC_LEFT = 1, LC_RIGHT = 2 };

  enum LaneChangeType {
    NONE = 0,
    MANUAL_LC = 1,
    ROUTING_LC = 2,
    EFFICIENT_LC = 3,
    BLOCKING_LC = 4
  };

  LaneChangeRequest lc_request;
  LaneChangeType lc_type;

  enum TargetReflineType {
    REF_CURRENT_LANE = 0,
    REF_LEFT_LANE = 1,
    REF_RIGHT_LANE = 2
  };

  TargetReflineType target_reference_line;

  // LC urgency definition:
  // Routing LC: function of time to diverge point
  // Manual/Efficient/Blocking LC: constant
  double lc_urgency;
  bool is_near_ramp;
  bool is_take_over;
  bool is_lc_ready;
  bool is_lc_abort;

  EnvInfos env_infos;
  LCFlags lc_flags;
  HazardVehInfo last_hazard_info;
  EfficentLaneInfo last_efficent_lane_info;

  void Reset() {
    this->lc_request = LaneChangeRequest::LANE_KEEP;
    this->lc_type = LaneChangeType::NONE;
    this->target_reference_line = TargetReflineType::REF_CURRENT_LANE;
    this->lc_urgency = 0.0;
    this->is_near_ramp = false;
    this->is_take_over = false;
    this->is_lc_ready = false;
    this->is_lc_abort = false;

    // DO NOT Reset lc_flags
    this->last_hazard_info.Reset();
    this->last_efficent_lane_info.Reset();
  }
};

}  // namespace planning
}  // namespace zark
