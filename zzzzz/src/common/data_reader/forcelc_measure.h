/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file forcelc_measure.h
 **/

#ifndef _FORCELC_MEASURE_H_
#define _FORCELC_MEASURE_H_

#include "apps/planning/src/config/conf_gflags.h"
#include "messages/planning/ads_decision.pb.h"

using namespace zark::ads_decision;
using namespace zark::ads_common;

namespace zark {
namespace planning {

typedef struct {
  // off ramp
  bool is_off_ramp_ = false;
  bool offramp_need_lc_ = false;
  float dist_off_ramp_start_ = DecisionGflags::alc_forcelc_distlim_valid;
  float dist_off_ramp_end_ = DecisionGflags::alc_forcelc_distlim_valid;
  int8_t target_split_lane_posn_ = 0;
  eLCRD_ForceLaneChgReqDir offramp_direct_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  bool is_offramp_complete_ = false;
} OffRampStr;

typedef struct {
  // on ramp
  bool is_on_ramp_ = false;
  bool onramp_need_lc_ = false;
  float dist_on_ramp_start_ = DecisionGflags::alc_forcelc_distlim_valid;
  float dist_on_ramp_end_ = DecisionGflags::alc_forcelc_distlim_valid;
  int8_t target_merge_lane_posn_ = 0;
  eLCRD_ForceLaneChgReqDir onramp_direct_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  bool is_onramp_complete_ = false;
} OnRampStr;

typedef struct {
  // avoid merging traffic flow
  bool is_avoid_traffic_flow_ = false;
  float dist_avoid_traffic_flow_start_ =
      DecisionGflags::alc_forcelc_distlim_valid;
  float dist_avoid_traffic_flow_end_ =
      DecisionGflags::alc_forcelc_distlim_valid;
  eLCRD_ForceLaneChgReqDir avoid_merge_lane_direct_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  ;
} AvoidMergeStr;

typedef struct {
  // other scenarios
  bool has_tunnel_ = false;
  bool has_traffic_jam_ = false;
  bool has_road_construct_ = false;
  bool has_toll_station_ = false;
  float dist_toll_station_start_ = DecisionGflags::alc_forcelc_distlim_valid;
  float dist_toll_station_end_ = DecisionGflags::alc_forcelc_distlim_valid;
  float sd_remain_distance_ = 0.0f;
  float hd_remain_distance_ = 0.0f;
} OtherScenarioStr;

typedef struct {
  // take over request
  eTOR_TakeOverReq take_over_request_ = eTOR_TakeOverReq::TOR_NoTakeover;
  eTORR_TakeOverReqRsn take_over_req_reason_ =
      eTORR_TakeOverReqRsn::TORR_NoTakeover;
  // 以下分别是一级接管和二级接管判断的变道次数，在非直通上下匝道的情况下两个变量有区别，
  // 以右侧下匝道为例：
  // 自车行驶在最右侧车道上，没驶入匝道前：
  // take_over_change_num_sec_ = 0，take_over_change_num_first_ = 1
  // 作用是自车在最右侧车道还没到匝道口前，抑制二级接管信号，
  // 如果接近驶入匝道的end, 仍未驶入匝道，此时会发一级接管请求
  int8_t take_over_change_num_sec_ = 0;
  int8_t take_over_change_num_first_ = 0;
  float dist_remain_ = DecisionGflags::alc_forcelc_distlim_valid;
} TakeOverStr;

typedef struct {
  // lane pos
  // 0 : current lane; 1 : left lane; 2 : secondary left lane;
  // -1 : right lane; -2 : secondary right lane;
  // 目标车道与当前车道位置关系
  int8_t dec_target_lane_posn_ = 0;
  eLCRR_ForceLaneChgReqRsn force_lc_reason_ =
      eLCRR_ForceLaneChgReqRsn::LCRR_NoReason;
  // 与最近的变道场景距离
  float dist_lc_senario_ = DecisionGflags::alc_forcelc_distlim_valid;
  // lane change dist parameter
  // 一般场景变道请求发起的距离阈值（与变道次数有关）
  float dist_general_lc_gate_ = 0.0f;
  // 拥堵/施工场景下的变道阈值
  float dist_jam_lc_gate_ = 0.0f;
  // 隧道场景下变道阈值
  float dist_tunnel_lc_gate_ = 0.0f;
  // 变道距离阈值（考虑一般变道阈值，拥堵、施工、隧道等阈值后的变道阈值）
  float dist_lc_gate_ = 0.0f;
  eLCRD_ForceLaneChgReqDir force_lc_direct_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  // force request time & reason hold
  uint64_t time_now_ = 0.0;
  // force请求一旦发送会持续至少2.0s （可标定），确保决策一致性
  eLCRR_ForceLaneChgReqRsn force_lc_reason_last_ =
      eLCRR_ForceLaneChgReqRsn::LCRR_NoReason;
  bool is_approach_ramp = false;
} ForceLcReqStr;

typedef struct {
  // ego info
  float ego_speed_ = DecisionGflags::alc_forcelc_speed_default;  // m/s
  float speed_limit_ = DecisionGflags::alc_forcelc_speed_default;
  int32_t ego_lanetype_ = 0;
  float full_connect_remain_dist_ = 0.0f;
  //   EHR_LaneType ego_lane_type_ = EHR_LaneType::LANE_TYPE_NORMAL;
  //   EHR_LaneType ego_left_lane_type_ = EHR_LaneType::LANE_TYPE_NORMAL;
  //   EHR_LaneType ego_right_lane_type_ = EHR_LaneType::LANE_TYPE_NORMAL;
  //   PlanMsg_PosnLane ego_lane_posn_ = PlanMsg_PosnLane::Pos_Unknown;
  int32_t current_seg_index_ = -1;  // speed limits extract from road segment.
  int8_t current_drive_lane_ = 0;
  int8_t lane_idx_from_left_ = 0;
  int8_t lane_idx_from_right_ = 0;
  float non_recommend_distance_ = DecisionGflags::alc_forcelc_distlim_valid;
  float remain_distance_ = DecisionGflags::alc_forcelc_distlim_valid;
  eLCRD_ForceLaneChgReqDir change_direction_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  uint8_t change_count_ = 0;
  uint8_t valid_lane_size_ = 0;
  bool is_near_ramp_ = false;
  bool cur_lane_merged_ = false;
  float dist_cur_lane_merged_ = 0;
  eLCRD_ForceLaneChgReqDir cur_merge_dir_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  bool cur_lane_split_ = false;
  float dist_cur_lane_split_ = 0;
  eLCRD_ForceLaneChgReqDir cur_split_dir_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  bool left_lane_merged_ = false;
  bool right_lane_merged_ = false;
  float dist_left_lane_merged_ = DecisionGflags::alc_forcelc_distlim_valid;
  float dist_right_lane_merged_ = DecisionGflags::alc_forcelc_distlim_valid;
  eLCRD_ForceLaneChgReqDir left_merge_dir_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  eLCRD_ForceLaneChgReqDir right_merge_dir_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  bool left_lane_recommend_ = false;
  bool right_lane_recommend_ = false;
  bool left_lane_split_ = false;
  bool right_lane_split_ = false;
  float dist_left_lane_split_ = DecisionGflags::alc_forcelc_distlim_valid;
  float dist_right_lane_split_ = DecisionGflags::alc_forcelc_distlim_valid;
  eLCRD_ForceLaneChgReqDir left_split_dir_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  eLCRD_ForceLaneChgReqDir right_split_dir_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  bool left_lane_merge_split_check_ = false;
  bool right_lane_merge_split_check_ = false;
  float left_boundary_dash2solid_dist_ = 0.0f;
  float right_boundary_dash2solid_dist_ = 0.0f;
  float left_lc_start_dist_ = 0.0f;
  float right_lc_start_dist_ = 0.0f;
  float left_lc_end_dist_ = 0.0f;
  float right_lc_end_dist_ = 0.0f;
  eLMT_LaneMrkrTyp left_boundary_type_ = eLMT_LaneMrkrTyp::LMT_None;
  eLMT_LaneMrkrTyp right_boundary_type_ = eLMT_LaneMrkrTyp::LMT_None;

} EgoInfoStr;

typedef struct {
  // inhibit
  int8_t inhibit_dir_ = 0;  // 0 no inhibit,1 left, 2 right, 3 all.
  int8_t last_inhibit_dir_ = 0;
  float reverse_inhibit_distance_ = DecisionGflags::alc_forcelc_distlim_avoid;
  int8_t left_change_count_ = 0;
  int8_t right_change_count_ = 0;
  eLCRD_ForceLaneChgReqDir left_change_dir_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  eLCRD_ForceLaneChgReqDir right_change_dir_ =
      eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  float left_remain_distance_ = 0.0f;
  float right_remain_distance_ = 0.0f;
  float left_inhibit_dist_gate_ = 0.0f;
  float right_inhibit_dist_gate_ = 0.0f;
  bool inhibit_left_lane_change_ = true;
  bool inhibit_right_lane_change_ = true;
} InhibitLcStr;

typedef struct {
  OffRampStr off_ramp_;
  OnRampStr on_ramp_;
  AvoidMergeStr avoid_merge_;
  OtherScenarioStr other_scenario_;
  TakeOverStr take_over_;
  ForceLcReqStr force_lc_;
  EgoInfoStr ego_info_;
  InhibitLcStr inhibit_lc_;
  OffRampStr other_split_ramp_;
  OnRampStr other_merge_ramp_;
} ForceLcMeasureInfo;

}  // namespace planning
}  // namespace zark

#endif  // _FORCELC_MEASURE_H_
