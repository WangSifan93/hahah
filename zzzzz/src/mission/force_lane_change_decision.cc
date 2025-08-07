/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file force_lane_change_decision.cc
 **/

#include "apps/planning/src/mission/force_lane_change_decision.h"

#include <string.h>

using namespace zark::ads_decision;
using namespace zark::ads_common;
using namespace zark::hdmap_new;
using zark::planning::LaneTopoInfo;
using zark::planning::NavigationInfo;

namespace zark {
namespace planning {

bool ForceLaneChangeDecision::DeciderProc(const NavigationInfo &navi_info,
                                          const DEC_Outputs &fitted_result) {
  if (!navi_info.valid_info) {
    Reset();
    env_model_output_.Clear();
    DataReader::GetInstance()->SetForceLCResult(env_model_output_);
    DataReader::GetInstance()->SetMeasureInfo(decision_info_);
    AERROR << "refline navi_info invalid, stop generate decision result.";
    return false;
  }
  const double check_time = zark::common::Clock::NowInSeconds();
  const double kMinTimeDiff = 2.0;
  if (check_time - navi_info.info_timestamp >= kMinTimeDiff) {
    Reset();
    env_model_output_.Clear();
    AERROR << "refline navi info stop update more than: "
           << check_time - navi_info.info_timestamp;
    return false;
  }
  UpdateEnvInfo(navi_info, decision_info_, last_route_info_);

  static uint8_t counter = 0;
  static uint8_t kMaxCounter = 15;
  //   env_model_output_.EM_OUT_DecRollingCounter_u8 = counter;
  counter++;
  counter = counter <= kMaxCounter ? counter : 0;
  decision_info_.force_lc_.time_now_ = zark::common::Clock::NowInSeconds();

  ScenarioProc(decision_info_.off_ramp_, decision_info_.on_ramp_,
               decision_info_.avoid_merge_, decision_info_.ego_info_, navi_info,
               decision_info_.force_lc_, last_onramp_);
  GetForceChangeLaneResult(navi_info, decision_info_);
  // force lane change req result
  AERROR << "---------force lane change req -------------";
  AERROR << "lane change direction: "
         << decision_info_.force_lc_.force_lc_direct_;

  // TODO: inhibit process
  InhibitProcess(last_onramp_, decision_info_);
  eLCRD_ForceLaneChgReqDir merge_dir = FindMergeLaneChangeDir(navi_info);
  FillForceLaneChgReqInfo(decision_info_, navi_info.noa_line, fitted_result,
                          merge_dir, env_model_output_);
  GetTakeOverResult(decision_info_);
  FillTakeOverReqInfo(decision_info_, env_model_output_);
  ContinueReqCheck(env_model_output_, last_req_keep_);
  IdentityMergeScenario(fitted_result, navi_info.change_points,
                        env_model_output_);

  env_model_output_.mutable_dec_out_hostlnelftline_bus()
      ->set_lli_stat_linetype_elmt(GetLeftBoundaryType());
  env_model_output_.mutable_dec_out_hostlnergtline_bus()
      ->set_lli_stat_linetype_elmt(GetRightBoundaryType());
  DataReader::GetInstance()->SetForceLCResult(env_model_output_);
  DataReader::GetInstance()->SetMeasureInfo(decision_info_);
  if (env_model_output_.dec_out_forcelanechgrslt_bus()
          .flcr_stat_forcelcddirt_elcrd() !=
      zark::ads_common::eLCRD_ForceLaneChgReqDir::LCRD_None_LC) {
    return true;
  } else {
    return false;
  }
}

void ForceLaneChangeDecision::IdentityMergeScenario(
    const DEC_Outputs &fitted_result,
    const std::vector<zark::hdmap_new::ChangePoint> &change_points,
    DEC_Outputs &dec_result) {
  if (!fitted_result.has_dec_out_curv_path_bus()) {
    return;
  }
  const auto &cur_path = fitted_result.dec_out_curv_path_bus();
  if (!cur_path.has_curv_nodes1_bus() || !cur_path.has_curv_nodes2_bus() ||
      !cur_path.has_curv_nodes3_bus()) {
    return;
  }
  const double kValidCurvature = 0.001;
  const double kMinFollowMergeDist = 200.0;
  if (fabs(cur_path.curv_nodes1_bus().curvature_val()) >= kValidCurvature ||
      fabs(cur_path.curv_nodes2_bus().curvature_val()) >= kValidCurvature ||
      fabs(cur_path.curv_nodes3_bus().curvature_val()) >= kValidCurvature) {
    return;
  }
  const float kDistanceCmToM = 0.01;
  for (auto point = change_points.begin(); point != change_points.end();
       point++) {
    if (point->type() ==
        hdmap_new::ChangePoint_Type::
            ChangePoint_Type_CHANGE_PT_TYPE_ROAD_ENTER_START_POINT) {
      if (fabs(point->distance_info().distance_to_vehicle_cm()) *
              kDistanceCmToM <=
          kMinFollowMergeDist) {
        auto next_point = std::next(point);
        if (next_point != change_points.end()) {
          if (next_point->type() ==
                  hdmap_new::ChangePoint_Type::
                      ChangePoint_Type_CHANGE_PT_TYPE_LANE_MERGE_START_POINT &&
              next_point->distance_info().distance_to_vehicle_cm() < 0.0 &&
              next_point->distance_info().distance_to_vehicle_cm() >
                  point->distance_info().distance_to_vehicle_cm()) {
            AINFO << "on ramp senario, but passed merge point!";
            break;
          }
        }
        AINFO << "on ramp senario, follow map line util merge main road!";
        dec_result.mutable_dec_out_noarampline_bus()->set_reserved(1);
      }
      break;
    }
  }
}

void ForceLaneChangeDecision::InhibitProcess(const OnRampDistKeep &last_onramp,
                                             ForceLcMeasureInfo &dec_info) {
  float left_inhibit_dist_gate = 0.0f;
  float right_inhibit_dist_gate = 0.0f;

  // update left/right lane remain distance with lc scenario
  if (dec_info.inhibit_lc_.left_change_count_ >
          dec_info.ego_info_.change_count_ &&
      dec_info.force_lc_.dist_lc_senario_ <
          dec_info.inhibit_lc_.left_remain_distance_) {
    dec_info.inhibit_lc_.left_remain_distance_ =
        dec_info.force_lc_.dist_lc_senario_;
  }
  if (dec_info.inhibit_lc_.right_change_count_ >
          dec_info.ego_info_.change_count_ &&
      dec_info.force_lc_.dist_lc_senario_ <
          dec_info.inhibit_lc_.right_remain_distance_) {
    dec_info.inhibit_lc_.right_remain_distance_ =
        dec_info.force_lc_.dist_lc_senario_;
  }

  // left lane change inhibit process
  if (dec_info.ego_info_.lane_idx_from_left_ >= 2) {
    if (dec_info.ego_info_.left_lane_recommend_ ||
        (!dec_info.ego_info_.left_lane_merged_ &&
         !dec_info.ego_info_.left_lane_split_ &&
         dec_info.ego_info_.change_direction_ ==
             eLCRD_ForceLaneChgReqDir::LCRD_Left_LC)) {
      dec_info.inhibit_lc_.inhibit_left_lane_change_ = false;
    } else if (!dec_info.ego_info_.left_lane_merged_ &&
               !dec_info.ego_info_.left_lane_split_) {
      left_inhibit_dist_gate = CalcuInhibitDistanceGate(
          dec_info.inhibit_lc_.left_change_count_,
          dec_info.ego_info_.ego_speed_, dec_info.ego_info_.speed_limit_);
      if (dec_info.inhibit_lc_.left_remain_distance_ > left_inhibit_dist_gate) {
        dec_info.inhibit_lc_.inhibit_left_lane_change_ = false;
      }
    }
  }

  // right lane change inhibit process
  if (dec_info.ego_info_.lane_idx_from_right_ >= 2 &&
      dec_info.ego_info_.valid_lane_size_ < 4 && !last_onramp.keep_onramp_) {
    if (dec_info.ego_info_.right_lane_recommend_ ||
        (!dec_info.ego_info_.right_lane_merged_ &&
         dec_info.ego_info_.change_direction_ ==
             eLCRD_ForceLaneChgReqDir::LCRD_Right_LC &&
         !dec_info.ego_info_.right_lane_split_)) {
      dec_info.inhibit_lc_.inhibit_right_lane_change_ = false;
    } else if (!dec_info.ego_info_.right_lane_merged_ &&
               !dec_info.ego_info_.right_lane_split_) {
      right_inhibit_dist_gate = CalcuInhibitDistanceGate(
          dec_info.inhibit_lc_.right_change_count_,
          dec_info.ego_info_.ego_speed_, dec_info.ego_info_.speed_limit_);
      if (dec_info.inhibit_lc_.right_remain_distance_ >
          right_inhibit_dist_gate) {
        dec_info.inhibit_lc_.inhibit_right_lane_change_ = false;
      }
    }
  } else if (dec_info.ego_info_.lane_idx_from_right_ >= 2 &&
             dec_info.ego_info_.valid_lane_size_ >= 4 &&
             !last_onramp.keep_onramp_) {
    if (dec_info.ego_info_.right_lane_recommend_ &&
        (!dec_info.ego_info_.right_lane_merged_) &&
        dec_info.ego_info_.change_direction_ ==
            eLCRD_ForceLaneChgReqDir::LCRD_Right_LC) {
      dec_info.inhibit_lc_.inhibit_right_lane_change_ = false;
    } else if (dec_info.ego_info_.lane_idx_from_right_ >= 3 &&
               !dec_info.ego_info_.right_lane_merged_ &&
               !dec_info.ego_info_.right_lane_split_) {
      right_inhibit_dist_gate = CalcuInhibitDistanceGate(
          dec_info.inhibit_lc_.right_change_count_,
          dec_info.ego_info_.ego_speed_, dec_info.ego_info_.speed_limit_);
      if (dec_info.inhibit_lc_.right_remain_distance_ >
          right_inhibit_dist_gate) {
        dec_info.inhibit_lc_.inhibit_right_lane_change_ = false;
      }
    }
  }

  ADEBUG << "--------- inhibit lane change ---------";
  ADEBUG << "inhibit_left_lane_change_ = "
         << dec_info.inhibit_lc_.inhibit_left_lane_change_;
  ADEBUG << "left_change_count_ = "
         << int(dec_info.inhibit_lc_.left_change_count_);
  ADEBUG << "left_remain_dist = " << dec_info.inhibit_lc_.left_remain_distance_;
  ADEBUG << "left_inhibit_dist_gate = " << left_inhibit_dist_gate;
  ADEBUG << "------------";
  ADEBUG << "inhibit_right_lane_change_ = "
         << dec_info.inhibit_lc_.inhibit_right_lane_change_;
  ADEBUG << "right_change_count_ = "
         << int(dec_info.inhibit_lc_.right_change_count_);
  ADEBUG << "right_remain_dist = "
         << dec_info.inhibit_lc_.right_remain_distance_;
  ADEBUG << "right_inhibit_dist_gate = " << right_inhibit_dist_gate;
}

float ForceLaneChangeDecision::CalcuInhibitDistanceGate(
    uint8_t lane_change_count, float vehicle_speed, float speed_limit) {
  float inhibit_distance = (lane_change_count + 1) *
                               DecisionGflags::alc_forcelc_time_lc *
                               DecisionGflags::alc_forcelc_coeff_lc *
                               std::max(speed_limit, vehicle_speed) +
                           DecisionGflags::alc_inhibit_dist_buffer;
  return inhibit_distance;
}

void ForceLaneChangeDecision::ContinueReqCheck(
    DEC_Outputs &env_model_output, LastReqKeepInfo &last_keep_info) {
  // current result;
  eLCRD_ForceLaneChgReqDir current_dir =
      env_model_output.dec_out_forcelanechgrslt_bus()
          .flcr_stat_forcelcddirt_elcrd();
  eLCRR_ForceLaneChgReqRsn current_reason =
      env_model_output.dec_out_forcelanechgrslt_bus()
          .flcr_stat_forcelcrsn_elcrr();
  // Decide whether to increase the corresponding count values based on whether
  // the current lane change direction is the same as the last lane change
  // direction.
  if (current_dir != eLCRD_ForceLaneChgReqDir::LCRD_None_LC) {
    if (current_dir == last_keep_info.last_direction) {
      // increase trigger count, set cancel count as 0;
      last_keep_info.force_trigger_cnt++;
      last_keep_info.force_cancel_cnt = 0;
    } else {
      // set trigger count as 0, and set last result as curren result;
      last_keep_info.last_direction = current_dir;
      last_keep_info.last_reason = current_reason;
      // determine whether increase cancel count depend on last req was sended;
      if (last_req_keep_.force_trigger_cnt < kMaxTriggerCnt) {
        // last send result is empty, not necessary add cancel cnt;
        last_keep_info.force_cancel_cnt = 0;
      } else {
        // last send result is explicit, need start add cancel cnt;
        last_keep_info.force_cancel_cnt++;
      }
      // last dir is not save as current dir, revert trigger cnt;
      last_keep_info.force_trigger_cnt = 0;
    }
  } else {
    // current dir is empty,
    if (last_keep_info.force_trigger_cnt < kMaxTriggerCnt) {
      // last send result is yet empty, revert last keep info;
      last_keep_info.last_direction = eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
      last_keep_info.last_reason = eLCRR_ForceLaneChgReqRsn::LCRR_NoReason;
      last_keep_info.force_cancel_cnt = 0;
      last_keep_info.force_trigger_cnt = 0;
    } else {
      // last has send explicit req , need keep last result, increase cancel
      // count, but not revert trigger cnt otherwise next cycle will set keep
      // info as empty;
      last_keep_info.force_cancel_cnt++;
    }
  }
  // count value limit;
  last_keep_info.force_cancel_cnt =
      last_keep_info.force_cancel_cnt >= kMaxCancelCnt
          ? kMaxCancelCnt
          : last_keep_info.force_cancel_cnt;
  last_keep_info.force_trigger_cnt =
      last_keep_info.force_trigger_cnt >= kMaxTriggerCnt
          ? kMaxTriggerCnt
          : last_keep_info.force_trigger_cnt;
  // count check
  if (last_keep_info.force_trigger_cnt > 0 &&
      last_keep_info.force_cancel_cnt == 0) {
    if (last_keep_info.force_trigger_cnt < kMaxTriggerCnt) {
      // current result has explict req,  but not meet continue check ,change to
      // empty result;
      env_model_output.mutable_dec_out_forcelanechgrslt_bus()
          ->set_flcr_stat_forcelcddirt_elcrd(
              eLCRD_ForceLaneChgReqDir::LCRD_None_LC);
      env_model_output.mutable_dec_out_forcelanechgrslt_bus()
          ->set_flcr_stat_forcelcrsn_elcrr(
              eLCRR_ForceLaneChgReqRsn::LCRR_NoReason);
      ADEBUG << "current_dir req is exist, but not meet continue check!!!  "
                "current dir: "
             << int(current_dir)
             << "  trigger_count: " << int(last_keep_info.force_trigger_cnt);
    } else {
      // send force req result
    }
  } else if (last_keep_info.force_cancel_cnt > 0) {
    if (last_keep_info.force_cancel_cnt >= kMaxCancelCnt) {
      // sent empty req result and set last result as empty;
      env_model_output.mutable_dec_out_forcelanechgrslt_bus()
          ->set_flcr_stat_forcelcddirt_elcrd(
              eLCRD_ForceLaneChgReqDir::LCRD_None_LC);
      env_model_output.mutable_dec_out_forcelanechgrslt_bus()
          ->set_flcr_stat_forcelcrsn_elcrr(
              eLCRR_ForceLaneChgReqRsn::LCRR_NoReason);
      last_keep_info.last_direction = eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
      last_keep_info.last_reason = eLCRR_ForceLaneChgReqRsn::LCRR_NoReason;
      // revert trigger cnt;
      last_keep_info.force_trigger_cnt = 0;
    } else {
      // cancel cnt is not meet continue check, need keep last result;
      env_model_output.mutable_dec_out_forcelanechgrslt_bus()
          ->set_flcr_stat_forcelcddirt_elcrd(last_keep_info.last_direction);
      env_model_output.mutable_dec_out_forcelanechgrslt_bus()
          ->set_flcr_stat_forcelcrsn_elcrr(last_keep_info.last_reason);
      ADEBUG << "current_dir is empty, but keep last req info, "
                "last_keep_info.last_direction  "
             << int(last_keep_info.last_direction);
    }
  } else {
    // trigger count and cancel count is 0, send empty result;
    env_model_output.mutable_dec_out_forcelanechgrslt_bus()
        ->set_flcr_stat_forcelcddirt_elcrd(
            eLCRD_ForceLaneChgReqDir::LCRD_None_LC);
    env_model_output.mutable_dec_out_forcelanechgrslt_bus()
        ->set_flcr_stat_forcelcrsn_elcrr(
            eLCRR_ForceLaneChgReqRsn::LCRR_NoReason);
  }
  ADEBUG << "trigger count: " << int(last_keep_info.force_trigger_cnt)
         << "  cancel count: " << int(last_keep_info.force_cancel_cnt)
         << "  last dir: " << int(last_keep_info.last_direction);
}

void ForceLaneChangeDecision::ScenarioProc(const OffRampStr &off_ramp,
                                           const OnRampStr &on_ramp,
                                           const AvoidMergeStr &avoid_merge,
                                           const EgoInfoStr &ego_info,
                                           const NavigationInfo &navi_info,
                                           ForceLcReqStr &force_lc,
                                           OnRampDistKeep &last_onramp) {
  // find closest lane change senario
  float lc_scenario_dist_buffer = 200.0;
  // off ramp lc scenario
  if (off_ramp.is_off_ramp_) {
    force_lc.dist_lc_senario_ = off_ramp.dist_off_ramp_start_;
    force_lc.dec_target_lane_posn_ = off_ramp.target_split_lane_posn_;
    force_lc.force_lc_reason_ = eLCRR_ForceLaneChgReqRsn::LCRR_EntrRamp;
    force_lc.force_lc_direct_ = off_ramp.offramp_direct_;

    ADEBUG << "------------lc scenario: off ramp";
    ADEBUG << "dist_lc_senario_ = " << force_lc.dist_lc_senario_;
    ADEBUG << "dec_target_lane_posn_ = " << int(force_lc.dec_target_lane_posn_);
    ADEBUG << "force_lc_direct_ = " << force_lc.force_lc_direct_;

    // ignore keep onramp distance
    if (!last_onramp.has_off_ramp_ &&
        off_ramp.dist_off_ramp_start_ <=
            DecisionGflags::alc_dist_offramp_ignkeeponramp) {
      last_onramp.Reset();
      last_onramp.has_off_ramp_ = true;

      ADEBUG << "----------last_onramp----------";
      ADEBUG << "has off ramp ";
    }
  }

  // on ramp lc scenario
  if (on_ramp.is_on_ramp_ &&
      on_ramp.dist_on_ramp_start_ <
          (force_lc.dist_lc_senario_ - lc_scenario_dist_buffer)) {
    force_lc.dist_lc_senario_ = on_ramp.dist_on_ramp_start_;
    force_lc.dec_target_lane_posn_ = on_ramp.target_merge_lane_posn_;
    force_lc.force_lc_reason_ = eLCRR_ForceLaneChgReqRsn::LCRR_EntrMainRoad;
    force_lc.force_lc_direct_ = on_ramp.onramp_direct_;

    ADEBUG << "------------lc scenario: on ramp";
    ADEBUG << "dist_lc_senario_ = " << force_lc.dist_lc_senario_;
    ADEBUG << "dec_target_lane_posn_ = " << int(force_lc.dec_target_lane_posn_);
    ADEBUG << "force_lc_direct_ = " << force_lc.force_lc_direct_;

    // set keep on ramp distance
    if (!last_onramp.keep_onramp_ && !last_onramp.has_off_ramp_ &&
        (on_ramp.dist_on_ramp_start_ <=
         DecisionGflags::alc_flc_dist_mergestart_buffer) &&
        (on_ramp.dist_on_ramp_start_ >
         -DecisionGflags::alc_flc_dist_mergestart_buffer)) {
      last_onramp.keep_onramp_ = true;
      last_onramp.keep_ramp_distance_ = -on_ramp.dist_on_ramp_start_;
      last_onramp.last_time_stamp_ = zark::common::Clock::NowInSeconds();

      AINFO << "----------last_onramp----------";
      AINFO << "start keep onramp!!! ";
    }
  }
  // on ramp lane change to second right lane
  eLCRD_ForceLaneChgReqDir merge_lane_change_dir =
      FindMergeLaneChangeDir(navi_info);
  AINFO << "merge_lane_change_dir " << merge_lane_change_dir;
  if (merge_lane_change_dir == eLCRD_ForceLaneChgReqDir::LCRD_Left_LC) {
    // set keep on ramp distance
    if (!last_onramp.keep_onramp_ && !last_onramp.has_off_ramp_ &&
        ego_info.remain_distance_ <=
            DecisionGflags::alc_flc_dist_mergestart_buffer) {
      last_onramp.keep_onramp_ = true;
      // last_onramp.keep_ramp_distance_ = -on_ramp.dist_on_ramp_start_;
      last_onramp.keep_ramp_distance_ = -ego_info.remain_distance_;
      last_onramp.last_time_stamp_ = zark::common::Clock::NowInSeconds();

      AINFO << "----------last_onramp----------";
      AINFO << "start keep onramp (lane merge)!!! ";
    }
  }

  // navigation lc scenario
  if (ego_info.change_count_ > 0 &&
      ego_info.remain_distance_ <
          (force_lc.dist_lc_senario_ - lc_scenario_dist_buffer)) {
    force_lc.dist_lc_senario_ =
        std::min(ego_info.non_recommend_distance_, ego_info.remain_distance_);
    force_lc.dec_target_lane_posn_ = ego_info.change_count_;
    force_lc.force_lc_reason_ = eLCRR_ForceLaneChgReqRsn::LCRR_Nav_LC;
    force_lc.force_lc_direct_ = ego_info.change_direction_;

    ADEBUG << "------------lc scenario: navi lc";
    ADEBUG << "dist_lc_senario_ (no recommend dist) = "
           << force_lc.dist_lc_senario_;
    ADEBUG << "dec_target_lane_posn_ = " << int(force_lc.dec_target_lane_posn_);
    ADEBUG << "force_lc_direct_ = " << force_lc.force_lc_direct_;
  }

  // avoid merge lc scenario
  if (avoid_merge.is_avoid_traffic_flow_ &&
      avoid_merge.avoid_merge_lane_direct_ !=
          eLCRD_ForceLaneChgReqDir::LCRD_None_LC &&
      avoid_merge.dist_avoid_traffic_flow_start_ <
          (force_lc.dist_lc_senario_ - lc_scenario_dist_buffer)) {
    force_lc.dist_lc_senario_ = avoid_merge.dist_avoid_traffic_flow_start_;
    force_lc.dec_target_lane_posn_ = 1;
    force_lc.force_lc_reason_ =
        eLCRR_ForceLaneChgReqRsn::LCRR_AviodMergTrffcFlow;
    force_lc.force_lc_direct_ = avoid_merge.avoid_merge_lane_direct_;
    ADEBUG << "------------lc scenario: avoid merge";
    ADEBUG << "dist_lc_senario_ = " << force_lc.dist_lc_senario_;
    ADEBUG << "dec_target_lane_posn_ = " << int(force_lc.dec_target_lane_posn_);
    ADEBUG << "force_lc_direct_ = " << force_lc.force_lc_direct_;
  }

  // reset keep onramp distance
  if (!off_ramp.is_off_ramp_ && last_onramp.has_off_ramp_) {
    last_onramp.has_off_ramp_ = false;

    ADEBUG << "----------last_onramp----------";
    ADEBUG << "clear has off ramp ";
  }

  // update keep onramp distance
  if (last_onramp.keep_onramp_) {
    double time_now_sec = zark::common::Clock::NowInSeconds();
    last_onramp.keep_ramp_distance_ +=
        ego_info.ego_speed_ * (time_now_sec - last_onramp.last_time_stamp_);
    last_onramp.last_time_stamp_ = time_now_sec;

    AINFO << "----------last_onramp----------";
    AINFO << "update: ";
    AINFO << "keep_ramp_distance_ = " << last_onramp.keep_ramp_distance_;
    AINFO << "last_time_stamp_ = " << last_onramp.last_time_stamp_;

    if (last_onramp.keep_ramp_distance_ >=
        DecisionGflags::alc_forcelc_dist_keep_onramp) {
      last_onramp.Reset();

      AINFO << "----------last_onramp----------";
      AINFO << "reset keep onramp over distance 1500 ";
    }
  }
}

eLCRD_ForceLaneChgReqDir ForceLaneChangeDecision::FindMergeLaneChangeDir(
    const NavigationInfo &navi_info) {
  // find current section
  std::string cur_section_id = navi_info.current_section_id;
  auto it_section =
      std::find_if(navi_info.sections.begin(), navi_info.sections.end(),
                   [cur_section_id](const NavSection &m) {
                     return m.section_id == cur_section_id;
                   });
  std::string lane_id = navi_info.current_lane_id;
  while (it_section != navi_info.sections.end()) {
    auto lanes = it_section->lanes;
    // find connect lane
    auto it_curlane = lanes.find(lane_id);
    if (it_curlane != lanes.end()) {
      LaneTopoInfo lane_info = it_curlane->second;
      uint8_t lane_seq = lane_info.lane_sequence;
      if (lane_info.is_recommend) {
        // find next connected section and lane
        auto section_topo = it_section->topo_range;
        auto it_next = section_topo.priority_sucessor_range.find(lane_seq);
        if (it_next == section_topo.priority_sucessor_range.end()) {
          return eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
        }
        std::string next_lane_id = it_next->second;
        // iterate next section and lane
        lane_id = next_lane_id;
        ++it_section;
      } else {
        // extract lane change info
        auto lane_transition = lane_info.lane_topo.transition();
        if (lane_transition == hdmap_new::Lane_LaneTransition::
                                   Lane_LaneTransition_LANE_TRANSITION_MERGE) {
          return lane_info.change_direction;
        } else {
          return eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
        }
      }
    } else {
      ++it_section;
    }
  }
  return eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
}

void ForceLaneChangeDecision::GetForceChangeLaneResult(
    const NavigationInfo &navi_info, ForceLcMeasureInfo &dec_info) {
  // calculate lane change dist gate
  LaneChgParaCacul(dec_info);

  ADEBUG << "dist_lc_gate_ = " << dec_info.force_lc_.dist_lc_gate_;

  // send lane change request
  if (dec_info.force_lc_.force_lc_reason_ !=
          eLCRR_ForceLaneChgReqRsn::LCRR_NoReason &&
      dec_info.force_lc_.dist_lc_senario_ <= dec_info.force_lc_.dist_lc_gate_) {
    ADEBUG << "------ Plan-SOC: Force lane change request ------";
    ADEBUG << "dist_lc_gate_ = " << dec_info.force_lc_.dist_lc_gate_;
    ADEBUG << "dist_lc_senario_ = " << dec_info.force_lc_.dist_lc_senario_;
    ADEBUG << "dec_target_lane_posn_ = "
           << int(dec_info.force_lc_.dec_target_lane_posn_);
    // if (dec_info.force_lc_.force_lc_direct_ ==
    //         eLCRD_ForceLaneChgReqDir::LCRD_Left_LC &&
    //     (!dec_info.ego_info_.left_lane_merged_ ||
    //      (dec_info.ego_info_.left_lane_merged_ &&
    //       dec_info.ego_info_.left_merge_dir_ !=
    //           eLCRD_ForceLaneChgReqDir::LCRD_Right_LC))) {
    if (dec_info.force_lc_.force_lc_direct_ ==
            eLCRD_ForceLaneChgReqDir::LCRD_Left_LC &&
        dec_info.ego_info_.left_lane_merge_split_check_) {
      if (dec_info.ego_info_.lane_idx_from_left_ < 2) {
        dec_info.force_lc_.force_lc_direct_ =
            eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
        ADEBUG << "------ can not left_lane_change: left_most lane ------";

        return;
      }
      ADEBUG << "LC direction: left";
      ADEBUG << "force_lc_reason_ : " << dec_info.force_lc_.force_lc_reason_;
      ADEBUG
          << "LCRR_EntrRamp = 1, EntrMainRoad = 2, Nav_LC = 3, LaneMerg = 4, "
             "AviodMergTrffcFlow = 5";
      return;

      // } else if (dec_info.force_lc_.force_lc_direct_ ==
      //                eLCRD_ForceLaneChgReqDir::LCRD_Right_LC &&
      //            (!dec_info.ego_info_.right_lane_merged_ ||
      //             (dec_info.ego_info_.right_lane_merged_ &&
      //              dec_info.ego_info_.right_merge_dir_ !=
      //                  eLCRD_ForceLaneChgReqDir::LCRD_Left_LC))) {
    } else if (dec_info.force_lc_.force_lc_direct_ ==
                   eLCRD_ForceLaneChgReqDir::LCRD_Right_LC &&
               dec_info.ego_info_.right_lane_merge_split_check_) {
      if (dec_info.ego_info_.lane_idx_from_right_ < 2) {
        dec_info.force_lc_.force_lc_direct_ =
            eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
        ADEBUG << "------ can not right_lane_change: right_most lane ------";

        return;
      }

      ADEBUG << "LC direction: right";
      ADEBUG << "force_lc_reason_ : " << dec_info.force_lc_.force_lc_reason_;
      ADEBUG
          << "LCRR_EntrRamp = 1, EntrMainRoad = 2, Nav_LC = 3, LaneMerg = 4, "
             "AviodMergTrffcFlow = 5";
      return;

    } else {
      dec_info.force_lc_.force_lc_direct_ =
          eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
      dec_info.force_lc_.force_lc_reason_ =
          eLCRR_ForceLaneChgReqRsn::LCRR_NoReason;
    }
  } else if (dec_info.ego_info_.remain_distance_ <=
             DecisionGflags::alc_onramp_dist_advanced_req) {
    // merge lane send lane change request advance
    eLCRD_ForceLaneChgReqDir merge_lane_change_dir =
        FindMergeLaneChangeDir(navi_info);
    float advance_lc_distance = 100.0f;
    float curlane_remain_distance = dec_info.ego_info_.remain_distance_ + 30.0;
    if (merge_lane_change_dir == eLCRD_ForceLaneChgReqDir::LCRD_Left_LC &&
        dec_info.inhibit_lc_.left_remain_distance_ >= curlane_remain_distance &&
        dec_info.ego_info_.left_lc_start_dist_ <= advance_lc_distance) {
      dec_info.force_lc_.force_lc_reason_ =
          eLCRR_ForceLaneChgReqRsn::LCRR_EntrMainRoad;
      dec_info.force_lc_.force_lc_direct_ = merge_lane_change_dir;
    } else if (merge_lane_change_dir ==
                   eLCRD_ForceLaneChgReqDir::LCRD_Right_LC &&
               dec_info.inhibit_lc_.right_remain_distance_ >=
                   curlane_remain_distance &&
               dec_info.ego_info_.right_lc_start_dist_ <= advance_lc_distance) {
      dec_info.force_lc_.force_lc_reason_ =
          eLCRR_ForceLaneChgReqRsn::LCRR_EntrMainRoad;
      dec_info.force_lc_.force_lc_direct_ = merge_lane_change_dir;
    }
  } else {
    dec_info.force_lc_.force_lc_direct_ =
        eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
    dec_info.force_lc_.force_lc_reason_ =
        eLCRR_ForceLaneChgReqRsn::LCRR_NoReason;
  }

  // on ramp lane change to second right lane
  float remain_dist_buffer = 1500.0f;
  if (last_onramp_.keep_onramp_ && last_onramp_.keep_ramp_distance_ > 0 &&
      last_onramp_.keep_ramp_distance_ <=
          DecisionGflags::alc_forcelc_dist_keep_onramp &&
      dec_info.ego_info_.lane_idx_from_right_ < 2 &&
      dec_info.ego_info_.lane_idx_from_left_ >= 3 &&
      dec_info.ego_info_.left_lane_merge_split_check_ &&
      (dec_info.inhibit_lc_.left_remain_distance_ >= remain_dist_buffer)) {
    dec_info.force_lc_.force_lc_direct_ =
        eLCRD_ForceLaneChgReqDir::LCRD_Left_LC;
    dec_info.force_lc_.force_lc_reason_ =
        eLCRR_ForceLaneChgReqRsn::LCRR_EntrMainRoad;

    ADEBUG << "------ Plan-SOC: Force lane change request ------";
    ADEBUG << "LC direction: left";
    ADEBUG << "force_lc_reason_ : " << dec_info.force_lc_.force_lc_reason_;
    ADEBUG << "LCRR_EntrRamp = 1, EntrMainRoad = 2, Nav_LC = 3, LaneMerg = 4, "
              "AviodMergTrffcFlow = 5";
  }
}

void ForceLaneChangeDecision::LaneChgParaCacul(ForceLcMeasureInfo &dec_info) {
  // calculate lane change num
  // int8_t lane_change_num = CalcuLaneChgNum(dec_info);
  int8_t lane_change_num = dec_info.force_lc_.dec_target_lane_posn_;

  if (lane_change_num > 0) {
    // calculate general lane change distance gate
    dec_info.force_lc_.dist_general_lc_gate_ =
        CalcuGeneralLCDistGate(dec_info, lane_change_num);

    dec_info.force_lc_.dist_lc_gate_ =
        std::max(dec_info.force_lc_.dist_general_lc_gate_,
                 std::max(dec_info.force_lc_.dist_jam_lc_gate_,
                          dec_info.force_lc_.dist_tunnel_lc_gate_));
  }
}

float ForceLaneChangeDecision::CalcuGeneralLCDistGate(
    const ForceLcMeasureInfo &dec_info, const int8_t lane_change_num) {
  float dist_gate = 0.0f;
  if (lane_change_num) {
    dist_gate =
        float(lane_change_num) *
            std::max(DecisionGflags::alc_forcelc_distlim_lc_min,
                     std::min(DecisionGflags::alc_forcelc_distlim_lc_max,
                              (DecisionGflags::alc_forcelc_time_lc *
                               DecisionGflags::alc_forcelc_coeff_lc *
                               std::max(dec_info.ego_info_.ego_speed_,
                                        dec_info.ego_info_.speed_limit_)))) +
        DecisionGflags::alc_forcelc_dist_buffer;
  }
  ADEBUG << "------ dist gate calcu ------";
  ADEBUG << "lane_change_num = " << int(lane_change_num);
  ADEBUG << "dec_info.ego_speed_ = " << dec_info.ego_info_.ego_speed_
         << "    dec_info.speed_limit_ = " << dec_info.ego_info_.speed_limit_;
  ADEBUG << "dist_gate = " << dist_gate;

  return dist_gate;
}

void ForceLaneChangeDecision::GetTakeOverResult(ForceLcMeasureInfo &dec_info) {
  // update remain distance info
  // UpdateRemainDist(ego_pos, dec_info);
  dec_info.take_over_.take_over_request_ = eTOR_TakeOverReq::TOR_NoTakeover;

  // first level take over request
  if (dec_info.other_scenario_.has_toll_station_ &&
      dec_info.other_scenario_.dist_toll_station_end_ >
          DecisionGflags::alc_flc_dist_tollend_buffer &&
      dec_info.other_scenario_.dist_toll_station_start_ <=
          DecisionGflags::alc_takeover_dist_fstlvl_toll) {
    dec_info.take_over_.take_over_req_reason_ =
        eTORR_TakeOverReqRsn::TORR_TollStationTakeOver;
    dec_info.take_over_.take_over_request_ =
        eTOR_TakeOverReq::TOR_FstLvl_Takeover;
  } else if (std::fabs(dec_info.other_scenario_.sd_remain_distance_) >
                 Epsilon &&
             dec_info.other_scenario_.sd_remain_distance_ <=
                 DecisionGflags::alc_takeover_dist_fstlvl_RtEnd) {
    dec_info.take_over_.take_over_req_reason_ =
        eTORR_TakeOverReqRsn::TORR_RouteEnd_Takeover;
    dec_info.take_over_.take_over_request_ =
        eTOR_TakeOverReq::TOR_FstLvl_Takeover;
  } else if (dec_info.other_scenario_.hd_remain_distance_ <
             DecisionGflags::alc_takeover_dist_fstlvl_RtEnd) {
    dec_info.take_over_.take_over_req_reason_ =
        eTORR_TakeOverReqRsn::TORR_RouteEnd_Takeover;
    dec_info.take_over_.take_over_request_ =
        eTOR_TakeOverReq::TOR_FstLvl_Takeover;
  } else if (dec_info.force_lc_.force_lc_reason_ ==
                 eLCRR_ForceLaneChgReqRsn::LCRR_EntrRamp &&
             dec_info.ego_info_.change_count_ > 0 &&
             dec_info.ego_info_.remain_distance_ <=
                 DecisionGflags::alc_takeover_dist_fstlvl_ramp) {
    dec_info.take_over_.take_over_req_reason_ =
        eTORR_TakeOverReqRsn::TORR_EntrRampTakeOver;
    dec_info.take_over_.take_over_request_ =
        eTOR_TakeOverReq::TOR_FstLvl_Takeover;
  } else if (dec_info.force_lc_.force_lc_reason_ ==
                 eLCRR_ForceLaneChgReqRsn::LCRR_EntrMainRoad &&
             dec_info.ego_info_.change_count_ > 0 &&
             dec_info.ego_info_.remain_distance_ <=
                 DecisionGflags::alc_takeover_dist_fstlvl_ramp) {
    dec_info.take_over_.take_over_req_reason_ =
        eTORR_TakeOverReqRsn::TORR_EntrMainRoadTakeOver;
    dec_info.take_over_.take_over_request_ =
        eTOR_TakeOverReq::TOR_FstLvl_Takeover;
  } else if (dec_info.ego_info_.remain_distance_ <=
             DecisionGflags::alc_takeover_dist_fstlvl_ramp) {
    // remain distance end take over
    dec_info.take_over_.take_over_req_reason_ =
        eTORR_TakeOverReqRsn::TORR_RouteEnd_Takeover;
    dec_info.take_over_.take_over_request_ =
        eTOR_TakeOverReq::TOR_FstLvl_Takeover;
  }

  if (dec_info.take_over_.take_over_request_ ==
      eTOR_TakeOverReq::TOR_FstLvl_Takeover) {
    ADEBUG << "take_over_request_: " << dec_info.take_over_.take_over_request_;
    ADEBUG << "take_over_change_num_first_ = "
           << dec_info.take_over_.take_over_change_num_first_;
    ADEBUG << "dist_remain_ = " << dec_info.take_over_.dist_remain_;
    return;
  }

  // secondary level take over
  if (dec_info.other_scenario_.has_toll_station_ &&
      dec_info.other_scenario_.dist_toll_station_end_ >
          DecisionGflags::alc_flc_dist_tollend_buffer &&
      dec_info.other_scenario_.dist_toll_station_start_ >
          DecisionGflags::alc_takeover_dist_fstlvl_toll &&
      dec_info.other_scenario_.dist_toll_station_start_ <=
          DecisionGflags::alc_takeover_dist_seclvl_toll) {
    dec_info.take_over_.take_over_req_reason_ =
        eTORR_TakeOverReqRsn::TORR_TollStationTakeOver;
    dec_info.take_over_.take_over_request_ =
        eTOR_TakeOverReq::TOR_SecLvl_Takeover;
  } else if (std::fabs(dec_info.other_scenario_.sd_remain_distance_) >
                 Epsilon &&
             dec_info.other_scenario_.sd_remain_distance_ >
                 DecisionGflags::alc_takeover_dist_fstlvl_RtEnd &&
             dec_info.other_scenario_.sd_remain_distance_ <=
                 DecisionGflags::alc_takeover_dist_seclvl_RtEnd) {
    dec_info.take_over_.take_over_req_reason_ =
        eTORR_TakeOverReqRsn::TORR_RouteEnd_Takeover;
    dec_info.take_over_.take_over_request_ =
        eTOR_TakeOverReq::TOR_SecLvl_Takeover;
  } else if (dec_info.other_scenario_.hd_remain_distance_ >
                 DecisionGflags::alc_takeover_dist_fstlvl_RtEnd &&
             dec_info.other_scenario_.hd_remain_distance_ <=
                 DecisionGflags::alc_takeover_dist_seclvl_RtEnd) {
    dec_info.take_over_.take_over_req_reason_ =
        eTORR_TakeOverReqRsn::TORR_RouteEnd_Takeover;
    dec_info.take_over_.take_over_request_ =
        eTOR_TakeOverReq::TOR_SecLvl_Takeover;
  } else if (dec_info.ego_info_.change_count_ > 0 &&
             dec_info.ego_info_.remain_distance_ >
                 DecisionGflags::alc_takeover_dist_fstlvl_ramp &&
             dec_info.ego_info_.remain_distance_ <=
                 (DecisionGflags::alc_takeover_dist_seclvl_ramp *
                  dec_info.ego_info_.change_count_)) {
    // send secondary level take over request
    dec_info.take_over_.take_over_request_ =
        eTOR_TakeOverReq::TOR_SecLvl_Takeover;
    if (dec_info.force_lc_.force_lc_reason_ ==
        eLCRR_ForceLaneChgReqRsn::LCRR_EntrRamp) {
      dec_info.take_over_.take_over_req_reason_ =
          eTORR_TakeOverReqRsn::TORR_EntrRampTakeOver;
    } else if (dec_info.force_lc_.force_lc_reason_ ==
               eLCRR_ForceLaneChgReqRsn::LCRR_EntrMainRoad) {
      dec_info.take_over_.take_over_req_reason_ =
          eTORR_TakeOverReqRsn::TORR_EntrMainRoadTakeOver;
    } else {
      // remain dist less than take over distance gate
      // road end take over
      dec_info.take_over_.take_over_req_reason_ =
          eTORR_TakeOverReqRsn::TORR_RouteEnd_Takeover;
    }
  }

  ADEBUG << "take_over_request_: " << dec_info.take_over_.take_over_request_;
  ADEBUG << "take_over_req_reason_ : "
         << dec_info.take_over_.take_over_req_reason_;
  ADEBUG << "take_over_change_num_sec_ = "
         << int(dec_info.ego_info_.change_count_);
  ADEBUG << "dist_remain_ = " << dec_info.ego_info_.remain_distance_;
  ADEBUG << "force_lc_reason_ = " << dec_info.force_lc_.force_lc_reason_;
}

void ForceLaneChangeDecision::UpdateMapInfo(
    const std::vector<ChangePoint> change_points,
    ForceLcMeasureInfo &dec_info) {
  // to do: match on/off ramp change point
  // float dist_match_tolerance = 50;
  float dist_remain = dec_info.ego_info_.remain_distance_;
  ADEBUG << "dist_remain = " << dist_remain;
  float offramp_remaindist_buffer = 300.0;

  // extract scenarios
  for (auto change_point : change_points) {
    ADEBUG << "dist_change_point = "
           << (change_point.distance_info().distance_to_vehicle_cm() / 100.0);
    ADEBUG << "change_point type = " << change_point.type();
    ADEBUG << "1: LANE_MERGE_START; 2: LANE_SPLIT_START; 3: ROAD_MERGE_START; ";
    ADEBUG << "5: ROAD_SPLIT_START; 7:ROAD_EXIT_START; 9:ROAD_ENTER_START ";
    if (change_point.type() ==
        hdmap_new::ChangePoint::Type::
            ChangePoint_Type_CHANGE_PT_TYPE_ROAD_EXIT_START_POINT) {
      float dist_split_point =
          change_point.distance_info().distance_to_vehicle_cm() / 100.0;
      if (dist_split_point >= DecisionGflags::alc_flc_dist_offrampend_buffer &&
          dist_remain < (dist_split_point + offramp_remaindist_buffer)) {
        dec_info.off_ramp_.is_off_ramp_ = true;
        dec_info.off_ramp_.dist_off_ramp_start_ = dist_split_point;
        dec_info.off_ramp_.dist_off_ramp_end_ =
            dec_info.ego_info_.remain_distance_;
        dec_info.off_ramp_.target_split_lane_posn_ =
            dec_info.ego_info_.change_count_;
        dec_info.off_ramp_.offramp_direct_ =
            dec_info.ego_info_.change_direction_;
        ADEBUG << "----------off ramp scenario: ";
        ADEBUG << "change count = "
               << int(dec_info.off_ramp_.target_split_lane_posn_);
        ADEBUG << "change_direction_ = "
               << int(dec_info.off_ramp_.offramp_direct_);
      }
      // complete off ramp
      float offramp_complt_start_dist = -10.0f;
      float offramp_complt_end_dist = -500.0f;
      if (dist_split_point <= offramp_complt_start_dist &&
          dist_split_point >= offramp_complt_end_dist) {
        if (dec_info.ego_info_.change_count_ == 0 ||
            dec_info.ego_info_.ego_lanetype_ &
                Lane::LaneType::Lane_LaneType_LANE_TYPE_NORMAL_LANE) {
          dec_info.off_ramp_.is_offramp_complete_ = true;
        }
      }

      // extract on ramp scenario
    } else if (change_point.type() ==
               hdmap_new::ChangePoint::Type::
                   ChangePoint_Type_CHANGE_PT_TYPE_ROAD_ENTER_START_POINT) {
      float dist_merge_point =
          change_point.distance_info().distance_to_vehicle_cm() / 100.0;
      if (dist_merge_point >= DecisionGflags::alc_flc_dist_onrampend_buffer) {
        dec_info.on_ramp_.is_on_ramp_ = true;
        dec_info.on_ramp_.dist_on_ramp_start_ = dist_merge_point;
        dec_info.on_ramp_.dist_on_ramp_end_ =
            dec_info.ego_info_.remain_distance_;
        dec_info.on_ramp_.target_merge_lane_posn_ =
            dec_info.ego_info_.change_count_;
        dec_info.on_ramp_.onramp_direct_ = dec_info.ego_info_.change_direction_;

        ADEBUG << "----------on ramp scenario: ";
        ADEBUG << "change count = "
               << int(dec_info.on_ramp_.target_merge_lane_posn_);
        ADEBUG << "change_direction_ = "
               << int(dec_info.on_ramp_.onramp_direct_);
      }
      // complete on ramp
      float onramp_complt_start_dist = -10.0f;
      float onramp_complt_end_dist = -500.0f;
      if (dist_merge_point <= onramp_complt_start_dist &&
          dist_merge_point >= onramp_complt_end_dist) {
        if (dec_info.ego_info_.change_count_ == 0 ||
            dec_info.ego_info_.ego_lanetype_ &
                Lane::LaneType::Lane_LaneType_LANE_TYPE_NORMAL_LANE) {
          dec_info.on_ramp_.is_onramp_complete_ = true;
        }
      }
    }
  }
  // avoid merge scenario
  if (dec_info.ego_info_.cur_lane_merged_ &&
      dec_info.ego_info_.dist_cur_lane_merged_ >=
          DecisionGflags::alc_avoid_distlim_lc_rampstart) {
    if (dec_info.off_ramp_.is_off_ramp_ &&
        (dec_info.off_ramp_.dist_off_ramp_end_ -
         dec_info.ego_info_.dist_cur_lane_merged_) <=
            DecisionGflags::alc_forcelc_dist_ign_mergramp) {
      ADEBUG << "ignore merging: too close to off ramp";
    } else if ((dec_info.ego_info_.cur_merge_dir_ ==
                    eLCRD_ForceLaneChgReqDir::LCRD_Left_LC &&
                dec_info.inhibit_lc_.left_remain_distance_ <=
                    DecisionGflags::alc_forcelc_avoidremdistmin) ||
               (dec_info.ego_info_.cur_merge_dir_ ==
                    eLCRD_ForceLaneChgReqDir::LCRD_Right_LC &&
                dec_info.inhibit_lc_.right_remain_distance_ <=
                    DecisionGflags::alc_forcelc_avoidremdistmin)) {
      ADEBUG << "ignore merging: avoid lane remain distance not enough";
    } else {
      dec_info.avoid_merge_.is_avoid_traffic_flow_ = true;
      dec_info.avoid_merge_.avoid_merge_lane_direct_ =
          dec_info.ego_info_.cur_merge_dir_;
      dec_info.avoid_merge_.dist_avoid_traffic_flow_start_ =
          dec_info.ego_info_.dist_cur_lane_merged_;
      ADEBUG << "----------avoid merge scenario: ";
      ADEBUG << "merge dir = "
             << dec_info.avoid_merge_.avoid_merge_lane_direct_;
      ADEBUG << "merge distance = "
             << dec_info.avoid_merge_.dist_avoid_traffic_flow_start_;
    }
    // avoid non-target off ramp split lane scenario
  } else if (dec_info.ego_info_.cur_lane_split_ &&
             dec_info.ego_info_.dist_cur_lane_split_ >=
                 DecisionGflags::alc_avoid_distlim_lc_rampstart) {
    if (!dec_info.avoid_merge_.is_avoid_traffic_flow_ ||
        (dec_info.avoid_merge_.is_avoid_traffic_flow_ &&
         dec_info.ego_info_.dist_cur_lane_split_ <
             dec_info.avoid_merge_.dist_avoid_traffic_flow_start_)) {
      if (dec_info.off_ramp_.is_off_ramp_ &&
          (dec_info.off_ramp_.dist_off_ramp_end_ -
           dec_info.ego_info_.dist_cur_lane_split_) <=
              DecisionGflags::alc_forcelc_dist_ign_mergramp) {
        ADEBUG
            << "ignore non-target off ramp split lane: too close to off ramp";
      } else if ((dec_info.ego_info_.cur_split_dir_ ==
                      eLCRD_ForceLaneChgReqDir::LCRD_Right_LC &&
                  dec_info.inhibit_lc_.left_remain_distance_ <=
                      DecisionGflags::alc_forcelc_avoidremdistmin) ||
                 (dec_info.ego_info_.cur_split_dir_ ==
                      eLCRD_ForceLaneChgReqDir::LCRD_Left_LC &&
                  dec_info.inhibit_lc_.right_remain_distance_ <=
                      DecisionGflags::alc_forcelc_avoidremdistmin)) {
        ADEBUG << "ignore merging: avoid lane remain distance not enough";
      } else {
        dec_info.avoid_merge_.is_avoid_traffic_flow_ = true;
        if (dec_info.ego_info_.cur_split_dir_ ==
            eLCRD_ForceLaneChgReqDir::LCRD_Left_LC) {
          dec_info.avoid_merge_.avoid_merge_lane_direct_ =
              eLCRD_ForceLaneChgReqDir::LCRD_Right_LC;
        } else if (dec_info.ego_info_.cur_split_dir_ ==
                   eLCRD_ForceLaneChgReqDir::LCRD_Right_LC) {
          dec_info.avoid_merge_.avoid_merge_lane_direct_ =
              eLCRD_ForceLaneChgReqDir::LCRD_Left_LC;
        }
        dec_info.avoid_merge_.dist_avoid_traffic_flow_start_ =
            dec_info.ego_info_.dist_cur_lane_split_;
        ADEBUG << "----------avoid merge scenario: ";
        ADEBUG << "avoid split dir = "
               << dec_info.avoid_merge_.avoid_merge_lane_direct_;
        ADEBUG << "avoid split distance = "
               << dec_info.avoid_merge_.dist_avoid_traffic_flow_start_;
      }
    }
  }
}

void ForceLaneChangeDecision::ResetMeasureRequest() {
  ForceLcReqStr empty_req_info;
  decision_info_.force_lc_ = empty_req_info;
}

void ForceLaneChangeDecision::CalcuFullConnectRemainDistance(
    const std::unordered_map<std::string, LaneTopoInfo> cur_lanes,
    EgoInfoStr &ego_info) {
  float min_remain_dist = DecisionGflags::alc_forcelc_distlim_valid;
  float min_split_dist = DecisionGflags::alc_forcelc_distlim_valid;
  for (auto lane : cur_lanes) {
    float lane_remain_dist = lane.second.remain_distance;
    float split_dist = lane.second.dist_split_end;
    min_remain_dist = std::min(min_remain_dist, lane_remain_dist);
    min_split_dist = std::min(min_split_dist, split_dist);
  }
  ego_info.full_connect_remain_dist_ =
      std::min(min_remain_dist, min_split_dist);
  ADEBUG << "full_connect_remain_dist_ = "
         << ego_info.full_connect_remain_dist_;
}

void ForceLaneChangeDecision::IsNearRampCheck(
    const NavigationInfo &navi_info,
    const std::unordered_map<std::string, zark::planning::LaneTopoInfo>
        cur_lanes,
    ForceLcMeasureInfo &dec_info) {
  float ramp_end_dist = DecisionGflags::alc_forcelc_distlim_valid;
  float kNearRampTimeGap = 10.0;
  // speed limit min value: 27.8 m/s or 100 km/h
  float kMinSpeedLimit = 27.8;
  float speed_limit = std::max(dec_info.ego_info_.speed_limit_, kMinSpeedLimit);
  // check on ramp scenario
  eLCRD_ForceLaneChgReqDir merge_dir = FindMergeLaneChangeDir(navi_info);
  if (merge_dir != eLCRD_ForceLaneChgReqDir::LCRD_None_LC) {
    ramp_end_dist = dec_info.ego_info_.remain_distance_;
    ADEBUG << "on ramp [near_ramp]: lane remain dist = " << ramp_end_dist;
    ADEBUG << "speed_limit = " << speed_limit;
    if (ramp_end_dist <= (kNearRampTimeGap * speed_limit)) {
      dec_info.ego_info_.is_near_ramp_ = true;
      return;
    }
  }
  // check current road is be merged
  bool is_road_merged = false;
  for (auto lane : cur_lanes) {
    if (lane.second.be_merged && lane.second.dist_merge_end < ramp_end_dist) {
      ramp_end_dist = lane.second.dist_merge_end;
      is_road_merged = true;
    }
  }
  if (is_road_merged && ramp_end_dist <= (kNearRampTimeGap * speed_limit)) {
    ADEBUG << "be merge [near_ramp]: lane remain dist = " << ramp_end_dist;
    ADEBUG << "speed_limit = " << speed_limit;
    dec_info.ego_info_.is_near_ramp_ = true;
    return;
  }
  // check off ramp scenario
  for (auto change_point : navi_info.change_points) {
    if (change_point.type() ==
        hdmap_new::ChangePoint::Type::
            ChangePoint_Type_CHANGE_PT_TYPE_ROAD_EXIT_START_POINT) {
      ramp_end_dist =
          change_point.distance_info().distance_to_vehicle_cm() / 100.0;
      if (ramp_end_dist <= (kNearRampTimeGap * speed_limit)) {
        ADEBUG << "off ramp EXIT [near_ramp]: lane remain dist = "
               << ramp_end_dist;
        ADEBUG << "speed_limit = " << speed_limit;
        dec_info.ego_info_.is_near_ramp_ = true;
        return;
      }
    }
    float kNearSplitTimeGap = 6.0;
    if (change_point.type() ==
        hdmap_new::ChangePoint::Type::
            ChangePoint_Type_CHANGE_PT_TYPE_ROAD_SPLIT_START_POINT) {
      ramp_end_dist =
          change_point.distance_info().distance_to_vehicle_cm() / 100.0;
      if (ramp_end_dist <= (kNearSplitTimeGap * speed_limit) &&
          ramp_end_dist >= (-kNearSplitTimeGap * speed_limit)) {
        ADEBUG << "off ramp SPLIT [near_ramp]: lane remain dist = "
               << ramp_end_dist;
        ADEBUG << "speed_limit = " << speed_limit;
        dec_info.ego_info_.is_near_ramp_ = true;
        return;
      }
    }
  }
}

void ForceLaneChangeDecision::UpdateEnvInfo(const NavigationInfo &navi_info,
                                            ForceLcMeasureInfo &dec_info,
                                            LastRouteInfo &last_route_info) {
  env_model_output_.Clear();
  // reset dec_info as default values
  ForceLcMeasureInfo default_dec_info{};
  dec_info = default_dec_info;
  dec_info.ego_info_.ego_speed_ = navi_info.ego_speed;
  // find current section
  std::string cur_section_id = navi_info.current_section_id;
  uint8_t current_lane_seq = 0;
  auto it_cur_section =
      std::find_if(navi_info.sections.begin(), navi_info.sections.end(),
                   [cur_section_id](const NavSection &m) {
                     return m.section_id == cur_section_id;
                   });
  if (it_cur_section != navi_info.sections.end()) {
    ADEBUG << "find current section!";
    auto cur_lanes = it_cur_section->lanes;
    dec_info.ego_info_.valid_lane_size_ = cur_lanes.size();

    ADEBUG << "current section id = " << cur_section_id;
    ADEBUG << "current lane id = " << navi_info.current_lane_id;

    // calculate remain distance when HDmap invalid
    CalcuFullConnectRemainDistance(cur_lanes, dec_info.ego_info_);

    // find current lane
    auto it_curlane = cur_lanes.find(navi_info.current_lane_id);
    if (it_curlane != cur_lanes.end()) {
      ADEBUG << "find current lane!";
      // extract lane info
      auto cur_lane_info = it_curlane->second;
      dec_info.ego_info_.lane_idx_from_right_ = cur_lane_info.lane_sequence;
      dec_info.ego_info_.lane_idx_from_left_ =
          (dec_info.ego_info_.valid_lane_size_ + 1) -
          cur_lane_info.lane_sequence;
      dec_info.ego_info_.change_count_ = cur_lane_info.change_count;
      dec_info.ego_info_.non_recommend_distance_ =
          cur_lane_info.non_recommend_distance;
      dec_info.ego_info_.remain_distance_ = cur_lane_info.remain_distance;
      // todo: remain dist measure proto
      dec_info.take_over_.dist_remain_ = cur_lane_info.remain_distance;

      dec_info.ego_info_.change_direction_ = cur_lane_info.change_direction;
      dec_info.ego_info_.speed_limit_ = cur_lane_info.speed_limit * kph2mps;
      dec_info.ego_info_.cur_lane_merged_ = cur_lane_info.be_merged;
      dec_info.ego_info_.dist_cur_lane_merged_ = cur_lane_info.dist_merge_end;
      dec_info.ego_info_.cur_merge_dir_ = cur_lane_info.merge_dir;
      current_lane_seq = cur_lane_info.lane_sequence;

      // current lane split info
      dec_info.ego_info_.cur_lane_split_ = cur_lane_info.be_split;
      dec_info.ego_info_.dist_cur_lane_split_ = cur_lane_info.dist_split_end;
      dec_info.ego_info_.cur_split_dir_ = cur_lane_info.split_dir;

      ADEBUG << "lane_idx_from_left_ = "
             << int(dec_info.ego_info_.lane_idx_from_left_);
      ADEBUG << "lane_idx_from_right_ = "
             << int(dec_info.ego_info_.lane_idx_from_right_);
    }

    // check is near ramp state
    IsNearRampCheck(navi_info, cur_lanes, dec_info);
    ADEBUG << "is_near_ramp_ = " << dec_info.ego_info_.is_near_ramp_;

    // find left/right lane
    int left_lane_seq = dec_info.ego_info_.lane_idx_from_right_ + 1;
    int right_lane_seq = dec_info.ego_info_.lane_idx_from_right_ - 1;
    float remain_dist_buffer = 50.0f;
    for (const auto &lane : cur_lanes) {
      if (lane.second.lane_sequence == left_lane_seq) {
        auto left_lane = lane.second;
        ADEBUG << "find left lane!";
        ADEBUG << "left.change_count = " << int(left_lane.change_count);
        ADEBUG << "left.remain_distance = " << left_lane.remain_distance;
        ADEBUG << "ego.remain_distance = "
               << dec_info.ego_info_.remain_distance_;
        // extract left lane info
        dec_info.ego_info_.left_lane_merged_ = left_lane.be_merged;
        dec_info.ego_info_.dist_left_lane_merged_ = left_lane.dist_merge_end;
        dec_info.ego_info_.left_merge_dir_ = left_lane.merge_dir;
        dec_info.inhibit_lc_.left_change_count_ = left_lane.change_count;
        dec_info.inhibit_lc_.left_remain_distance_ = left_lane.remain_distance;
        dec_info.inhibit_lc_.left_change_dir_ = left_lane.change_direction;
        // left lane split info
        dec_info.ego_info_.left_lane_split_ = left_lane.be_split;
        dec_info.ego_info_.dist_left_lane_split_ = left_lane.dist_split_end;
        dec_info.ego_info_.left_split_dir_ = left_lane.split_dir;
        // left lane recommond check
        if (dec_info.ego_info_.lane_idx_from_left_ >= 2 &&
            left_lane.change_count <= dec_info.ego_info_.change_count_ &&
            (left_lane.remain_distance >=
             (dec_info.ego_info_.remain_distance_ - remain_dist_buffer)) &&
            (!dec_info.ego_info_.left_lane_merged_ ||
             (dec_info.ego_info_.left_lane_merged_ &&
              dec_info.ego_info_.left_merge_dir_ !=
                  eLCRD_ForceLaneChgReqDir::LCRD_Right_LC)) &&
            (!dec_info.ego_info_.left_lane_split_ ||
             (dec_info.ego_info_.left_lane_split_ &&
              dec_info.ego_info_.left_split_dir_ !=
                  eLCRD_ForceLaneChgReqDir::LCRD_Left_LC))) {
          dec_info.ego_info_.left_lane_recommend_ = true;
        }
        // left lane merge/split check
        // merge lane check
        if ((!dec_info.ego_info_.left_lane_merged_ ||
             (dec_info.ego_info_.left_lane_merged_ &&
              dec_info.ego_info_.left_merge_dir_ !=
                  eLCRD_ForceLaneChgReqDir::LCRD_Right_LC)) ||
            ((dec_info.ego_info_.remain_distance_ -
              dec_info.ego_info_.dist_cur_lane_merged_) <=
             DecisionGflags::alc_forcelc_dist_ign_mergramp)) {
          // split lane check
          if ((!dec_info.ego_info_.left_lane_split_ ||
               (dec_info.ego_info_.left_lane_split_ &&
                dec_info.ego_info_.left_split_dir_ !=
                    eLCRD_ForceLaneChgReqDir::LCRD_Left_LC)) ||
              ((dec_info.ego_info_.remain_distance_ -
                dec_info.ego_info_.dist_cur_lane_split_) <=
               DecisionGflags::alc_forcelc_dist_ign_mergramp)) {
            dec_info.ego_info_.left_lane_merge_split_check_ = true;
          }
        }

      } else if (lane.second.lane_sequence == right_lane_seq) {
        auto right_lane = lane.second;
        ADEBUG << "find right lane!";
        ADEBUG << "right.change_count = " << int(right_lane.change_count);
        ADEBUG << "right.remain_distance = " << right_lane.remain_distance;
        ADEBUG << "ego.remain_distance = "
               << dec_info.ego_info_.remain_distance_;
        // extract right lane info
        dec_info.ego_info_.right_lane_merged_ = right_lane.be_merged;
        dec_info.ego_info_.dist_right_lane_merged_ = right_lane.dist_merge_end;
        dec_info.ego_info_.right_merge_dir_ = right_lane.merge_dir;
        dec_info.inhibit_lc_.right_change_count_ = right_lane.change_count;
        dec_info.inhibit_lc_.right_remain_distance_ =
            right_lane.remain_distance;
        dec_info.inhibit_lc_.right_change_dir_ = right_lane.change_direction;
        // right lane split info
        dec_info.ego_info_.right_lane_split_ = right_lane.be_split;
        dec_info.ego_info_.dist_right_lane_split_ = right_lane.dist_split_end;
        dec_info.ego_info_.right_split_dir_ = right_lane.split_dir;
        if (dec_info.ego_info_.lane_idx_from_right_ >= 2 &&
            right_lane.change_count <= dec_info.ego_info_.change_count_ &&
            (right_lane.remain_distance >=
             (dec_info.ego_info_.remain_distance_ - remain_dist_buffer)) &&
            (!dec_info.ego_info_.right_lane_merged_ ||
             (dec_info.ego_info_.right_lane_merged_ &&
              dec_info.ego_info_.right_merge_dir_ !=
                  eLCRD_ForceLaneChgReqDir::LCRD_Left_LC)) &&
            (!dec_info.ego_info_.right_lane_split_ ||
             (dec_info.ego_info_.right_lane_split_ &&
              dec_info.ego_info_.right_split_dir_ !=
                  eLCRD_ForceLaneChgReqDir::LCRD_Right_LC))) {
          dec_info.ego_info_.right_lane_recommend_ = true;
        }

        // right lane merge/split check
        // merge lane check
        if ((!dec_info.ego_info_.right_lane_merged_ ||
             (dec_info.ego_info_.right_lane_merged_ &&
              dec_info.ego_info_.right_merge_dir_ !=
                  eLCRD_ForceLaneChgReqDir::LCRD_Left_LC)) ||
            ((dec_info.ego_info_.remain_distance_ -
              dec_info.ego_info_.dist_cur_lane_merged_) <=
             DecisionGflags::alc_forcelc_dist_ign_mergramp)) {
          // split lane check
          if ((!dec_info.ego_info_.right_lane_split_ ||
               (dec_info.ego_info_.right_lane_split_ &&
                dec_info.ego_info_.right_split_dir_ !=
                    eLCRD_ForceLaneChgReqDir::LCRD_Right_LC)) ||
              ((dec_info.ego_info_.remain_distance_ -
                dec_info.ego_info_.dist_cur_lane_split_) <=
               DecisionGflags::alc_forcelc_dist_ign_mergramp)) {
            dec_info.ego_info_.right_lane_merge_split_check_ = true;
          }
        }
      }
    }
  }

  ADEBUG << "-------------ego_info --------------";
  ADEBUG << "change_count_ = " << int(dec_info.ego_info_.change_count_);
  ADEBUG << "change_direction_ = " << dec_info.ego_info_.change_direction_;

  ADEBUG << "right_lane_merged_ = " << dec_info.ego_info_.right_lane_merged_;
  ADEBUG << "right_merge_dir_ = " << dec_info.ego_info_.right_merge_dir_;
  ADEBUG << "right_lane_split_ = " << dec_info.ego_info_.right_lane_split_;
  ADEBUG << "right_lane_recommend_ = "
         << dec_info.ego_info_.right_lane_recommend_;

  ADEBUG << "left_lane_merged_ = " << dec_info.ego_info_.left_lane_merged_;
  ADEBUG << "left_merge_dir_ = " << dec_info.ego_info_.left_merge_dir_;
  ADEBUG << "left_lane_recommend_ = "
         << dec_info.ego_info_.left_lane_recommend_;
  AINFO << "left_lane_merge_split_check_ = "
        << dec_info.ego_info_.left_lane_merge_split_check_;
  AINFO << "right_lane_merge_split_check_ = "
        << dec_info.ego_info_.right_lane_merge_split_check_;

  auto map_data = DataReader::GetInstance()->GetMapData();
  // extract toll station info
  auto map_sections = map_data.guidance_info.section();
  ExtractSectionInfo(map_sections, navi_info, dec_info);
  // extract route end info
  hdmap_new::NavigationInfo map_navi_info = map_data.navigation_info;
  if (map_navi_info.match_status() ==
      hdmap_new::NavigationInfo::MatchStatus::
          NavigationInfo_MatchStatus_MAT_STATUS_MATCH_OK) {
    // navigation route end distance
    dec_info.other_scenario_.sd_remain_distance_ =
        map_navi_info.sd_remain_distance_m();
    // hd map remain distance
    dec_info.other_scenario_.hd_remain_distance_ =
        map_navi_info.hd_remain_distance_m();
    // update last route distance info
    last_route_info.hd_distance = map_navi_info.hd_remain_distance_m();
    last_route_info.sd_distance = map_navi_info.sd_remain_distance_m();
  } else {
    // navigation route end distance
    dec_info.other_scenario_.sd_remain_distance_ = last_route_info.sd_distance;
    // hd map remain distance
    dec_info.other_scenario_.hd_remain_distance_ = last_route_info.hd_distance;
  }
  // extract map info
  UpdateMapInfo(navi_info.change_points, dec_info);
  if (it_cur_section != navi_info.sections.end() && current_lane_seq != 0) {
    UpdateLineInfo(navi_info.sections, it_cur_section, current_lane_seq,
                   dec_info);
    // process map lane info
    auto map_lanes = map_data.guidance_info.lane();
    std::unordered_map<std::string, hdmap_new::LaneInfo> lanes_topo;
    for (auto &guidance_lane : map_lanes) {
      std::string map_lane_id = guidance_lane.id().id();
      if (lanes_topo.count(map_lane_id) == 0) {
        lanes_topo.emplace(map_lane_id, guidance_lane);
      } else {
        AWARN << "exist repeat lane in guidance info: " << map_lane_id;
      }
    }
    UpdateLeftBoundaryInfo(lanes_topo, navi_info.sections,
                           navi_info.current_lane_id, dec_info);
    UpdateRightBoundaryInfo(lanes_topo, navi_info.sections,
                            navi_info.current_lane_id, dec_info);
  }
}

void ForceLaneChangeDecision::ExtractSectionInfo(
    const google::protobuf::RepeatedPtrField<zark::hdmap_new::SectionInfo>
        &road_segments,
    const NavigationInfo &navi_info, ForceLcMeasureInfo &dec_info) {
  // find nearest toll station
  for (hdmap_new::SectionInfo section : road_segments) {
    float section_start_dist =
        section.distance_info().distance_to_vehicle_cm() * Kcenti2meter;
    if (section.attribute() & RoadSection::RoadAttribute::
                                  RoadSection_RoadAttribute_ATTR_TOLLSTATION &&
        section_start_dist <
            dec_info.other_scenario_.dist_toll_station_start_) {
      dec_info.other_scenario_.has_toll_station_ = true;
      dec_info.other_scenario_.dist_toll_station_start_ = section_start_dist;
      dec_info.other_scenario_.dist_toll_station_end_ =
          (section.distance_info().distance_to_vehicle_cm() +
           section.distance_info().end_offset_cm() -
           section.distance_info().start_offset_cm()) *
          Kcenti2meter;
      ADEBUG << "---------has toll station----------";
      ADEBUG << "dist_toll_station_start_ = "
             << dec_info.other_scenario_.dist_toll_station_start_;
      ADEBUG << "dist_toll_station_end_ = "
             << dec_info.other_scenario_.dist_toll_station_end_;
      ADEBUG << "section id = " << section.id().id();
    }
  }
  // find roads before toll station without lane markers
  // (lane num >= 6)
  if (dec_info.other_scenario_.has_toll_station_ &&
      dec_info.on_ramp_.dist_on_ramp_end_ >
          dec_info.other_scenario_.dist_toll_station_start_) {
    for (hdmap_new::SectionInfo section : road_segments) {
      float section_start_dist =
          section.distance_info().distance_to_vehicle_cm() * Kcenti2meter;
      int8_t lane_num = section.lane_id_size();
      float search_dist = 300.0;
      if ((section_start_dist >=
           (dec_info.other_scenario_.dist_toll_station_start_ - search_dist)) &&
          (section_start_dist <
           dec_info.other_scenario_.dist_toll_station_start_) &&
          (lane_num >= 6)) {
        dec_info.other_scenario_.dist_toll_station_start_ = section_start_dist;
      }
    }
  }

  if (navi_info.special_section_ids.size() > 0) {
    if (navi_info.special_section_ids.count(navi_info.current_section_id)) {
      dec_info.other_scenario_.dist_toll_station_start_ = 1.0;
    } else {
      bool find_final = false;
      for (const auto &final_section : road_segments) {
        if (find_final) {
          break;
        }
        for (const auto &next_section : final_section.successor_id()) {
          if (navi_info.special_section_ids.count(next_section.id())) {
            float final_distance =
                (final_section.distance_info().distance_to_vehicle_cm() +
                 final_section.distance_info().end_offset_cm() -
                 final_section.distance_info().start_offset_cm()) *
                Kcenti2meter;
            dec_info.other_scenario_.dist_toll_station_start_ =
                (dec_info.other_scenario_.dist_toll_station_start_ <=
                 final_distance)
                    ? dec_info.other_scenario_.dist_toll_station_start_
                    : final_distance;
            find_final = true;
            ADEBUG << " revise toll station start distance: "
                   << dec_info.other_scenario_.dist_toll_station_start_
                   << "  section start : "
                   << final_section.distance_info().start_offset_cm()
                   << "  end_offset : "
                   << final_section.distance_info().end_offset_cm()
                   << "  to ego : "
                   << final_section.distance_info().distance_to_vehicle_cm();
            ADEBUG << "section id = " << final_section.id().id();
            break;
          }
        }
      }
    }
  }
}

void ForceLaneChangeDecision::UpdateLeftBoundaryInfo(
    const std::unordered_map<std::string, hdmap_new::LaneInfo> lanes_topo,
    const std::list<NavSection> &road_segments,
    const std::string current_lane_id, ForceLcMeasureInfo &dec_info) {
  // float linetype_dist_buffer = 20.0f;
  float ego_pose_buffer = 10.0;
  float find_dist_buffer = 10.0;
  // find left boundary dash to solid distance
  float max_search_distance = 1000.0f;
  float left_boundary_search_dist = 0.0f;
  float left_boundary2solid_dist = 0.0f;
  float left_boundary2dash_dist = 0.0f;
  float lc_space_end_distance = 0.0f;
  bool find_dash2solid = false;
  bool find_solid2dash = false;
  bool need_find_lc_end = false;
  std::string lane_id = current_lane_id;
  for (auto it_section = road_segments.begin();
       it_section != road_segments.end(); ++it_section) {
    if (left_boundary_search_dist >= max_search_distance) {
      ADEBUG << "more than 300";
      if (!find_dash2solid) {
        left_boundary2solid_dist = max_search_distance;
      }
      if (!find_solid2dash) {
        left_boundary2dash_dist = max_search_distance;
      }
      if (lc_space_end_distance <= Epsilon) {
        lc_space_end_distance = max_search_distance;
      }
      break;
    }

    // find ego lane or connected lane
    auto it_lane = lanes_topo.find(lane_id);
    if (it_lane != lanes_topo.end()) {
      hdmap_new::LaneInfo lane_info = it_lane->second;
      auto left_boundary = lane_info.left_boundary_attr();
      // find current lane left boundary
      for (auto it_boundary = left_boundary.begin();
           it_boundary != left_boundary.end(); ++it_boundary) {
        left_boundary_search_dist =
            (it_boundary->distance_info().distance_to_vehicle_cm() +
             (it_boundary->distance_info().end_offset_cm() -
              it_boundary->distance_info().start_offset_cm())) *
            Kcenti2meter;

        float boundary_start_dist =
            (it_boundary->distance_info().distance_to_vehicle_cm()) *
            Kcenti2meter;
        ADEBUG << "boundary_start_dist = " << boundary_start_dist;
        LaneBoundaryAttribute_Type boundary_type = it_boundary->type();
        ADEBUG << "lane_type:" << int(boundary_type);
        // find current lane left boudary type
        if ((boundary_start_dist <= (ego_pose_buffer + find_dist_buffer)) &&
            (left_boundary_search_dist >
             (ego_pose_buffer - find_dist_buffer))) {
          dec_info.ego_info_.left_boundary_type_ =
              FillBoundaryType(boundary_type);
          ADEBUG << "left_boundary_type_ = "
                 << dec_info.ego_info_.left_boundary_type_;
        }
        ADEBUG << "left_boundary_search_dist = " << left_boundary_search_dist;

        if (left_boundary_search_dist < ego_pose_buffer) {
          continue;
        }

        if (boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DENSE_WIDE_DASH ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DOUBLE_DASH ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LEFT_SOLID_AND_RIGHT_DASH ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LONG_DASHED_LINE ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SHORT_DASHED_LINE ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SINGLE_DASHED_LINE ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_NO_MARKING) {
          if (!find_solid2dash) {
            left_boundary2dash_dist = std::max(
                (it_boundary->distance_info().distance_to_vehicle_cm() *
                 Kcenti2meter),
                0.0f);
            // left_boundary2dash_dist =
            //     (left_boundary2dash_dist >= linetype_dist_buffer)
            //         ? left_boundary2dash_dist
            //         : 0.0f;
            find_solid2dash = true;
            if (find_dash2solid &&
                left_boundary2solid_dist < left_boundary2dash_dist) {
              need_find_lc_end = true;
            }
          }
        } else if (!find_dash2solid) {
          left_boundary2solid_dist =
              std::max((it_boundary->distance_info().distance_to_vehicle_cm() *
                        Kcenti2meter),
                       0.0f);
          // left_boundary2solid_dist =
          //     (left_boundary2solid_dist >= linetype_dist_buffer)
          //         ? left_boundary2solid_dist
          //         : 0.0f;
          // lc_space_end_distance = left_boundary2solid_dist;
          find_dash2solid = true;
          ADEBUG << "find solid line";
        } else if (need_find_lc_end && (lc_space_end_distance <= Epsilon)) {
          lc_space_end_distance =
              std::max((it_boundary->distance_info().distance_to_vehicle_cm() *
                        Kcenti2meter),
                       0.0f);
        }
      }
      auto lanes = it_section->lanes;
      auto nav_it_lane = lanes.find(lane_id);
      if (nav_it_lane == lanes.end()) {
        AERROR << "left lane boundary: cannot get next lane from navi_info";
        // if (!find_dash2solid) {
        //   left_boundary2solid_dist = left_boundary_search_dist;
        // }
        // if (!find_solid2dash) {
        //   left_boundary2dash_dist = left_boundary_search_dist;
        // }
        // if (lc_space_end_distance <= Epsilon) {
        //   lc_space_end_distance = left_boundary_search_dist;
        // }
        continue;
      }

      uint8_t lane_seq = nav_it_lane->second.lane_sequence;
      ADEBUG << "lane_seq = " << lane_seq;
      auto section_topo = it_section->topo_range;
      auto it_next = section_topo.priority_sucessor_range.find(lane_seq);
      if (it_next == section_topo.priority_sucessor_range.end()) {
        ADEBUG << "not has next connect lane";
        if (!find_dash2solid) {
          left_boundary2solid_dist = left_boundary_search_dist;
        }
        if (!find_solid2dash) {
          left_boundary2dash_dist = left_boundary_search_dist;
        }
        if (lc_space_end_distance <= Epsilon) {
          lc_space_end_distance = left_boundary_search_dist;
        }
        break;
      }
      std::string next_lane_id = it_next->second;
      // iterate next section and lane
      lane_id = next_lane_id;
    } else {
      ADEBUG << "not find lane";
      break;
    }
  }
  left_boundary2solid_dist = (left_boundary2solid_dist > max_search_distance)
                                 ? max_search_distance
                                 : left_boundary2solid_dist;

  left_boundary2dash_dist = (left_boundary2dash_dist > max_search_distance)
                                ? max_search_distance
                                : left_boundary2dash_dist;

  lc_space_end_distance = (lc_space_end_distance > max_search_distance)
                              ? max_search_distance
                              : lc_space_end_distance;

  dec_info.ego_info_.left_boundary_dash2solid_dist_ = left_boundary2solid_dist;
  dec_info.ego_info_.left_lc_start_dist_ = left_boundary2dash_dist;
  dec_info.ego_info_.left_lc_end_dist_ = lc_space_end_distance;
  ADEBUG << "left_boundary_dash2solid_dist_ = "
         << dec_info.ego_info_.left_boundary_dash2solid_dist_;
  ADEBUG << "left_lc_start_dist_ = " << dec_info.ego_info_.left_lc_start_dist_;
  ADEBUG << "left_lc_end_dist_ = " << dec_info.ego_info_.left_lc_end_dist_;
}

void ForceLaneChangeDecision::UpdateRightBoundaryInfo(
    const std::unordered_map<std::string, hdmap_new::LaneInfo> lanes_topo,
    const std::list<NavSection> &road_segments,
    const std::string current_lane_id, ForceLcMeasureInfo &dec_info) {
  // float linetype_dist_buffer = 20.0f;
  float ego_pose_buffer = 10.0;
  float find_dist_buffer = 10.0;
  // find right boundary dash to solid distance
  float max_search_distance = 1000.0f;
  float right_boundary_search_dist = 0.0f;
  float right_boundary2solid_dist = 0.0f;
  float right_boundary2dash_dist = 0.0f;
  float lc_space_end_distance = 0.0f;
  bool find_dash2solid = false;
  bool find_solid2dash = false;
  bool need_find_lc_end = false;
  std::string lane_id = current_lane_id;
  for (auto it_section = road_segments.begin();
       it_section != road_segments.end(); ++it_section) {
    if (right_boundary_search_dist >= max_search_distance) {
      ADEBUG << "more than 300";
      if (!find_dash2solid) {
        right_boundary2solid_dist = max_search_distance;
      }
      if (!find_solid2dash) {
        right_boundary2dash_dist = max_search_distance;
      }
      if (lc_space_end_distance <= Epsilon) {
        lc_space_end_distance = max_search_distance;
      }
      break;
    }

    // find ego lane or connected lane
    auto it_lane = lanes_topo.find(lane_id);
    if (it_lane != lanes_topo.end()) {
      hdmap_new::LaneInfo lane_info = it_lane->second;
      auto right_boundary = lane_info.right_boundary_attr();
      // find current lane right boundary
      for (auto it_boundary = right_boundary.begin();
           it_boundary != right_boundary.end(); ++it_boundary) {
        right_boundary_search_dist =
            (it_boundary->distance_info().distance_to_vehicle_cm() +
             (it_boundary->distance_info().end_offset_cm() -
              it_boundary->distance_info().start_offset_cm())) *
            Kcenti2meter;

        float boundary_start_dist =
            (it_boundary->distance_info().distance_to_vehicle_cm()) *
            Kcenti2meter;
        ADEBUG << "boundary_start_dist = " << boundary_start_dist;
        LaneBoundaryAttribute_Type boundary_type = it_boundary->type();
        ADEBUG << "lane_type:" << int(boundary_type);
        // find current lane right boudary type
        if ((boundary_start_dist <= (ego_pose_buffer + find_dist_buffer)) &&
            (right_boundary_search_dist >
             (ego_pose_buffer - find_dist_buffer))) {
          dec_info.ego_info_.right_boundary_type_ =
              FillBoundaryType(boundary_type);
          ADEBUG << "right_boundary_type_ = "
                 << dec_info.ego_info_.right_boundary_type_;
        }
        ADEBUG << "right_boundary_search_dist = " << right_boundary_search_dist;

        if (right_boundary_search_dist < ego_pose_buffer) {
          continue;
        }

        if (boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DENSE_WIDE_DASH ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DOUBLE_DASH ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_RIGHT_SOLID_AND_LEFT_DASH ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LONG_DASHED_LINE ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SHORT_DASHED_LINE ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SINGLE_DASHED_LINE ||
            boundary_type ==
                LaneBoundaryAttribute_Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_NO_MARKING) {
          if (!find_solid2dash) {
            right_boundary2dash_dist = std::max(
                (it_boundary->distance_info().distance_to_vehicle_cm() *
                 Kcenti2meter),
                0.0f);
            // right_boundary2dash_dist =
            //     (right_boundary2dash_dist >= linetype_dist_buffer)
            //         ? right_boundary2dash_dist
            //         : 0.0f;
            find_solid2dash = true;
            if (find_dash2solid &&
                right_boundary2solid_dist < right_boundary2dash_dist) {
              need_find_lc_end = true;
            }
          }
        } else if (!find_dash2solid) {
          right_boundary2solid_dist =
              std::max((it_boundary->distance_info().distance_to_vehicle_cm() *
                        Kcenti2meter),
                       0.0f);
          // right_boundary2solid_dist =
          //     (right_boundary2solid_dist >= linetype_dist_buffer)
          //         ? right_boundary2solid_dist
          //         : 0.0f;
          lc_space_end_distance = right_boundary2solid_dist;
          find_dash2solid = true;
          ADEBUG << "find solid line";
        } else if (need_find_lc_end && (lc_space_end_distance <= Epsilon)) {
          lc_space_end_distance =
              std::max((it_boundary->distance_info().distance_to_vehicle_cm() *
                        Kcenti2meter),
                       0.0f);
        }
      }
      auto lanes = it_section->lanes;
      auto nav_it_lane = lanes.find(lane_id);
      if (nav_it_lane == lanes.end()) {
        AERROR << "right lane boundary: cannot get next lane from navi_info";
        continue;
      }

      uint8_t lane_seq = nav_it_lane->second.lane_sequence;
      ADEBUG << "lane_seq = " << lane_seq;
      auto section_topo = it_section->topo_range;
      auto it_next = section_topo.priority_sucessor_range.find(lane_seq);
      if (it_next == section_topo.priority_sucessor_range.end()) {
        ADEBUG << "not has next connect lane";
        if (!find_dash2solid) {
          right_boundary2solid_dist = right_boundary_search_dist;
        }
        if (!find_solid2dash) {
          right_boundary2dash_dist = right_boundary_search_dist;
        }
        if (lc_space_end_distance <= Epsilon) {
          lc_space_end_distance = right_boundary_search_dist;
        }
        break;
      }
      std::string next_lane_id = it_next->second;
      // iterate next section and lane
      lane_id = next_lane_id;
    } else {
      ADEBUG << "not find lane";
      if (!find_dash2solid) {
        right_boundary2solid_dist = right_boundary_search_dist;
      }
      if (!find_solid2dash) {
        right_boundary2dash_dist = right_boundary_search_dist;
      }
      if (lc_space_end_distance <= Epsilon) {
        lc_space_end_distance = right_boundary_search_dist;
      }
      break;
    }
  }

  right_boundary2solid_dist = (right_boundary2solid_dist > max_search_distance)
                                  ? max_search_distance
                                  : right_boundary2solid_dist;

  right_boundary2dash_dist = (right_boundary2dash_dist > max_search_distance)
                                 ? max_search_distance
                                 : right_boundary2dash_dist;

  lc_space_end_distance = (lc_space_end_distance > max_search_distance)
                              ? max_search_distance
                              : lc_space_end_distance;

  dec_info.ego_info_.right_boundary_dash2solid_dist_ =
      right_boundary2solid_dist;
  dec_info.ego_info_.right_lc_start_dist_ = right_boundary2dash_dist;
  dec_info.ego_info_.right_lc_end_dist_ = lc_space_end_distance;
  ADEBUG << "right_boundary_dash2solid_dist_ = "
         << dec_info.ego_info_.right_boundary_dash2solid_dist_;
  ADEBUG << "right_lc_start_dist_ = "
         << dec_info.ego_info_.right_lc_start_dist_;
  ADEBUG << "right_lc_end_dist_ = " << dec_info.ego_info_.right_lc_end_dist_;
}

void ForceLaneChangeDecision::UpdateLineInfo(
    const std::list<NavSection> &road_segments,
    const std::list<NavSection>::const_iterator &current_seg,
    const uint8_t current_lane_seq, const ForceLcMeasureInfo &dec_info) {
  // first segment dis select according to scenario
  float fix_dis = DecisionGflags::alc_speed_limt_first_dis;  // 200m
  if (dec_info.off_ramp_.is_off_ramp_ && dec_info.on_ramp_.is_on_ramp_) {
    if (dec_info.on_ramp_.dist_on_ramp_start_ <
        dec_info.off_ramp_.dist_off_ramp_start_) {
      fix_dis = DecisionGflags::alc_speed_limt_merge_main_road_dis;  // 50m
    } else {
      fix_dis = DecisionGflags::alc_speed_limt_enter_ramp_dis;  // 200m
    }
  } else if (dec_info.off_ramp_.is_off_ramp_) {
    fix_dis = DecisionGflags::alc_speed_limt_enter_ramp_dis;  // 200m

  } else if (dec_info.on_ramp_.is_on_ramp_) {
    fix_dis = DecisionGflags::alc_speed_limt_merge_main_road_dis;  // 50m
  }
  // Process based on the remaining length of current road_segment
  float select_speed = current_seg->max_speed_limit;
  if (current_seg->end_s >= fix_dis) {
    // remaining >= 100m, current_segment speed_limit is used as
    // first_section speed_limit.
    B1_SpeedLimitInfo *host_speed_1 =
        env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
            ->mutable_dec_out_v_spdlmt1_bus();
    host_speed_1->set_speed_limit(current_seg->max_speed_limit);
    host_speed_1->set_start_s(0.0);
    host_speed_1->set_end_s(fix_dis);
    // Extract speed_limits for the remaining three sections
    ExtendSpeedLimits(road_segments, current_seg, fix_dis, current_seg->end_s,
                      env_model_output_);
  } else {
    // remaining < 100m, search forward segments utill distance > 100,
    // use minmum speed limit as first_section speed_limit.
    // PlanMsg_RoadSegment start_seg;
    auto next_seg = std::next(current_seg);
    float accumulate_dis = current_seg->end_s;
    while (next_seg != road_segments.end()) {
      select_speed = std::fmin(select_speed, next_seg->max_speed_limit);
      accumulate_dis += (next_seg->end_s - next_seg->start_s);
      if (accumulate_dis >= fix_dis) {
        // start_seg = road_segments[i];
        B1_SpeedLimitInfo *host_speed_1 =
            env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
                ->mutable_dec_out_v_spdlmt1_bus();
        host_speed_1->set_speed_limit(select_speed);
        host_speed_1->set_start_s(0.0);
        host_speed_1->set_end_s(fix_dis);
        // Extract speed_limits for the remaining three sections
        ExtendSpeedLimits(road_segments, next_seg, fix_dis, accumulate_dis,
                          env_model_output_);
        break;
      }
      next_seg++;
    }
  }

  // Fill other speed limit information based on the speed_limits
  // size mentioned above and ego pos.
  FillSpeedLimitsInfo(current_seg->valid_lane_size, current_lane_seq,
                      env_model_output_);
}

void ForceLaneChangeDecision::ExtendSpeedLimits(
    const std::list<NavSection> &road_segments,
    const std::list<NavSection>::const_iterator &start_seg,
    const float &fix_dis, const float &accumulate_dis, DEC_Outputs &output) {
  float dis_buffer = DecisionGflags::alc_speed_limt_dis_buffer;  // 100m
  const int8_t kRemainSectionNum = 4;  // remaining segments
  if (std::next(start_seg) == road_segments.end()) {
    if (accumulate_dis > fix_dis) {
      B1_SpeedLimitInfo *host_speed_2 =
          env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
              ->mutable_dec_out_v_spdlmt2_bus();
      host_speed_2->set_start_s(fix_dis);
      host_speed_2->set_end_s(accumulate_dis);
      host_speed_2->set_speed_limit(start_seg->max_speed_limit);
    }
    return;
  }
  // Consider whether the remaining distance of the first segment is divided
  // by 100 meters is sufficient
  bool remain_sufficient = false;
  if (!remain_sufficient && accumulate_dis >= fix_dis + dis_buffer) {
    B1_SpeedLimitInfo *host_speed_i =
        env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
            ->mutable_dec_out_v_spdlmt2_bus();
    host_speed_i->set_start_s(fix_dis);
    host_speed_i->set_end_s(accumulate_dis);
    host_speed_i->set_speed_limit(start_seg->max_speed_limit);
    remain_sufficient = true;
  }
  float end_s = accumulate_dis;
  float accu_start_s = accumulate_dis;
  auto next_seg = std::next(start_seg);
  for (int8_t j = 1; j <= kRemainSectionNum; j++) {
    if (next_seg == road_segments.end()) {
      break;
    }
    const float seg_length = next_seg->end_s - next_seg->start_s;
    end_s += seg_length;
    if (j == 1) {
      // extracting second speed_section, it is necessary to consider that the
      // remaining distance of the first segment divided by 100 meters is
      // sufficient, use remaining distance as second segment.
      B1_SpeedLimitInfo *host_speed_i =
          remain_sufficient
              ? env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
                    ->mutable_dec_out_v_spdlmt3_bus()
              : env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
                    ->mutable_dec_out_v_spdlmt2_bus();
      float start_s = remain_sufficient ? accu_start_s : fix_dis;
      host_speed_i->set_start_s(start_s);
      host_speed_i->set_end_s(end_s);
      host_speed_i->set_speed_limit(next_seg->max_speed_limit);
    } else {
      if (j == 2) {
        B1_SpeedLimitInfo *host_speed_i =
            remain_sufficient
                ? env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
                      ->mutable_dec_out_v_spdlmt4_bus()
                : env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
                      ->mutable_dec_out_v_spdlmt3_bus();
        host_speed_i->set_start_s(accu_start_s);
        host_speed_i->set_end_s(end_s);
        host_speed_i->set_speed_limit(next_seg->max_speed_limit);
      } else if (j == 3) {
        B1_SpeedLimitInfo *host_speed_i =
            remain_sufficient
                ? env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
                      ->mutable_dec_out_v_spdlmt5_bus()
                : env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
                      ->mutable_dec_out_v_spdlmt4_bus();
        host_speed_i->set_start_s(accu_start_s);
        host_speed_i->set_end_s(end_s);
        host_speed_i->set_speed_limit(next_seg->max_speed_limit);
        if (remain_sufficient) break;
      } else if (j == 4) {
        B1_SpeedLimitInfo *host_speed_i =
            env_model_output_.mutable_dec_out_v_hostlnespdlmt_bus()
                ->mutable_dec_out_v_spdlmt5_bus();
        host_speed_i->set_start_s(accu_start_s);
        host_speed_i->set_end_s(end_s);
        host_speed_i->set_speed_limit(next_seg->max_speed_limit);
      }
    }
    accu_start_s += seg_length;
    next_seg++;
  }
}

ads_common::eLMT_LaneMrkrTyp ForceLaneChangeDecision::FillBoundaryType(
    const hdmap_new::LaneBoundaryAttribute_Type &boundary_type) {
  ads_common::eLMT_LaneMrkrTyp out_line_type = eLMT_LaneMrkrTyp::LMT_None;
  switch (boundary_type) {
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_UNKNOWN:
      out_line_type = ads_common::eLMT_LaneMrkrTyp::LMT_Unknown;
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_NO_MARKING:
      out_line_type = ads_common::eLMT_LaneMrkrTyp::LMT_None;
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LONG_DASHED_LINE:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SHORT_DASHED_LINE:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SINGLE_DASHED_LINE:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DENSE_WIDE_DASH:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DIAMOND_DECELERATION_LINE:
      out_line_type = ads_common::eLMT_LaneMrkrTyp::LMT_Dashed;
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DOUBLE_DASH:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DOUBLE_DIAMOND_DECELERATION:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LEFT_DASHED_AND_RIGHT_DIAMOND_DECELERATION:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_RIGHT_DASHED_AND_LEFT_DIAMOND_DECELERATION:
      out_line_type = ads_common::eLMT_LaneMrkrTyp::LMT_DoubleDashedLine;
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SINGLE_SOLID_LINE:
      out_line_type = ads_common::eLMT_LaneMrkrTyp::LMT_Solid;
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DOUBLE_SOLID:
      out_line_type = ads_common::eLMT_LaneMrkrTyp::LMT_DoubleSolidLine;
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LEFT_SOLID_AND_RIGHT_DASH:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LEFT_SOLID_AND_RIGHT_DIAMOND_DECELERATION:
      out_line_type = ads_common::eLMT_LaneMrkrTyp::LMT_LeftSolidRightDashed;
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_RIGHT_SOLID_AND_LEFT_DASH:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_RIGHT_SOLID_AND_LEFT_DIAMOND_DECELERATION:
      out_line_type = ads_common::eLMT_LaneMrkrTyp::LMT_RightSolidLeftDashed;
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_CURB:
      out_line_type = ads_common::eLMT_LaneMrkrTyp::LMT_Curbstone;
      break;
    default:
      out_line_type = ads_common::eLMT_LaneMrkrTyp::LMT_None;
      break;
  }
  return out_line_type;
}

void ForceLaneChangeDecision::FillSpeedLimitsInfo(
    const uint8_t valid_lane_size, const uint8_t current_lane_seq,
    DEC_Outputs &env_model_output) {
  // Determine whether it is necessary to set the left and right lane speed
  // limits to empty based on the current lane positioning information
  if (valid_lane_size <= 1) {
    return;
  }
  if (current_lane_seq == valid_lane_size) {
    // ego pos is Left_mostLane, set left_lane speed info as all 0;
    env_model_output.mutable_dec_out_v_rgtlnespdlmt_bus()->CopyFrom(
        env_model_output.dec_out_v_hostlnespdlmt_bus());
  } else if (current_lane_seq == 1) {
    // ego pos is Right_mostLane, set right_lane speed info as all 0;
    env_model_output.mutable_dec_out_v_lftlnespdlmt_bus()->CopyFrom(
        env_model_output.dec_out_v_hostlnespdlmt_bus());
  } else {
    // in middle lane, fill left and right speed limit same as host lane
    env_model_output.mutable_dec_out_v_rgtlnespdlmt_bus()->CopyFrom(
        env_model_output.dec_out_v_hostlnespdlmt_bus());
    env_model_output.mutable_dec_out_v_lftlnespdlmt_bus()->CopyFrom(
        env_model_output.dec_out_v_hostlnespdlmt_bus());
  }
}

void ForceLaneChangeDecision::FillForceLaneChgReqInfo(
    const ForceLcMeasureInfo &dec_info, const NoaLine noa_line,
    const DEC_Outputs &fitted_result, eLCRD_ForceLaneChgReqDir merge_dir,
    DEC_Outputs &env_model_output) {
  // get localization data
  localization::LocalizationInfo local_data =
      DataReader::GetInstance()->GetLocalizationData();
  mapfusion::MapLocalizationInfo maplocal_data =
      DataReader::GetInstance()->GetMapState();
  planning::MapInfo map_data = DataReader::GetInstance()->GetMapData();
  // fill state info
  auto state_info = env_model_output.mutable_dec_out_stateinfo_bus();
  state_info->set_dec_out_stat_decfuncstate_edfs(
      zark::ads_common::eDFS_DecFuncState::DFS_DecFunc_ON);
  // map
  switch (map_data.map_type) {
    case hdmap_new::MapHeader::MapType::MapHeader_MapType_MAP_TYPE_HDPRO:
      state_info->set_dec_out_stat_mapmodel_emm(
          zark::ads_common::eMM_MapModel::MM_HDMAP_MODEL);
      break;
    case hdmap_new::MapHeader::MapType::MapHeader_MapType_MAP_TYPE_SD_HDPRO:
      state_info->set_dec_out_stat_mapmodel_emm(
          zark::ads_common::eMM_MapModel::MM_HDMAP_MODEL);
      break;
    case hdmap_new::MapHeader::MapType::MapHeader_MapType_MAP_TYPE_SD:
      state_info->set_dec_out_stat_mapmodel_emm(
          zark::ads_common::eMM_MapModel::MM_SDMAP_MODEL);
      break;

    default:
      state_info->set_dec_out_stat_mapmodel_emm(
          zark::ads_common::eMM_MapModel::MM_INVALID_MODEL);
      break;
  }
  // map road class
  std::string cur_section_id = maplocal_data.road_section_id();
  auto it_cur_section =
      std::find_if(map_data.guidance_info.road().begin(),
                   map_data.guidance_info.road().end(),
                   [cur_section_id](const hdmap_new::RoadInfo &m) {
                     return m.id().id() == cur_section_id;
                   });
  if (it_cur_section != map_data.guidance_info.road().end()) {
    ADEBUG << "find current section!";
    switch (it_cur_section->road_class()) {
      case hdmap_new::Road_RoadClass::Road_RoadClass_RC_EXPRESSWAY:
        state_info->set_dec_out_stat_roadtype_ert(
            ads_common::eRT_RoadType::RT_highway);
        break;
      case hdmap_new::Road_RoadClass::Road_RoadClass_RC_URBAN_EXPRESSWAY:
        state_info->set_dec_out_stat_roadtype_ert(
            ads_common::eRT_RoadType::RT_highway);
        break;

      default:
        state_info->set_dec_out_stat_roadtype_ert(
            ads_common::eRT_RoadType::RT_urban);
        break;
    }
  }
  // map navigation state
  hdmap_new::NavigationInfo map_navi_info = map_data.navigation_info;
  switch (map_navi_info.nav_status()) {
    case hdmap_new::NavigationInfo::NAV_STATUS_NAVIGATION:
      state_info->set_dec_out_stat_routetype_ertt(
          zark::ads_common::eRTT_RouteType::RTT_Navigation);
      break;
    case hdmap_new::NavigationInfo::NAV_STATUS_PATH_REPLANNING:
      state_info->set_dec_out_stat_routetype_ertt(
          zark::ads_common::eRTT_RouteType::RTT_Rerouting);
      break;

    default:
      state_info->set_dec_out_stat_routetype_ertt(
          zark::ads_common::eRTT_RouteType::RTT_Invalid);
      break;
  }
  // loc func state
  localization::LocalizationState loc_func_state = local_data.loc_state();
  state_info->set_dec_out_stat_locstate_els(
      static_cast<ads_common::eLS_LocStateType>(loc_func_state));

  // loc map match state
  if (maplocal_data.has_state()) {
    mapfusion::MapLocalizationState maploc_match_state =
        maplocal_data.state().state();
    state_info->set_dec_out_stat_locmatchstate_elms(
        static_cast<ads_common::eLMS_LocMatchStateType>(maploc_match_state));
  }

  // fill ramp info
  auto ramp_info = env_model_output.mutable_dec_out_rampinfo_bus();
  // on ramp success
  ramp_info->set_dec_out_is_onrampcomplt_bl(
      dec_info.on_ramp_.is_onramp_complete_);
  // off ramp success
  ramp_info->set_dec_out_is_offrampcomplt_bl(
      dec_info.off_ramp_.is_offramp_complete_);

  // fill distance info
  auto dist_info = env_model_output.mutable_dec_out_distanceinfo_bus();
  // current lane remain distance
  dist_info->set_dec_out_d_offrouteremaindist_sg(
      dec_info.ego_info_.remain_distance_);
  // full connect remain distance
  dist_info->set_dec_out_d_cruiseremaindist_sg(
      dec_info.ego_info_.full_connect_remain_dist_);
  // navigation route end distance
  dist_info->set_dec_out_d_routenddist_sg(
      dec_info.other_scenario_.sd_remain_distance_);
  // hd map remain distance
  float hd_map_distance = dec_info.other_scenario_.hd_remain_distance_;
  if (dec_info.other_scenario_.has_toll_station_ &&
      dec_info.other_scenario_.dist_toll_station_end_ >=
          DecisionGflags::alc_flc_dist_tollend_buffer) {
    hd_map_distance =
        (hd_map_distance <= dec_info.other_scenario_.dist_toll_station_start_)
            ? hd_map_distance
            : dec_info.other_scenario_.dist_toll_station_start_;
    hd_map_distance = (hd_map_distance >= 0.0) ? hd_map_distance : 0.0f;
  }
  dist_info->set_dec_out_d_hdmapremaindist_sg(hd_map_distance);

  // merge lane remain distance
  if (merge_dir != eLCRD_ForceLaneChgReqDir::LCRD_None_LC) {
    dist_info->set_dec_out_d_merglaneremaindist_sg(
        dec_info.ego_info_.remain_distance_);
  } else {
    dist_info->set_dec_out_d_merglaneremaindist_sg(
        DecisionGflags::alc_forcelc_distlim_valid);
  }
  // fill remain distance to ramp info
  if (dec_info.off_ramp_.is_off_ramp_) {
    dist_info->set_dec_out_d_remaindst2rmp_sg(
        dec_info.off_ramp_.dist_off_ramp_start_);
  } else {
    dist_info->set_dec_out_d_remaindst2rmp_sg(
        DecisionGflags::alc_forcelc_distlim_valid);
  }
  // fill remain distance to main road info
  if (dec_info.on_ramp_.is_on_ramp_) {
    dist_info->set_dec_out_d_remaindst2mainrd_sg(
        dec_info.on_ramp_.dist_on_ramp_start_);
  } else {
    dist_info->set_dec_out_d_remaindst2mainrd_sg(
        DecisionGflags::alc_forcelc_distlim_valid);
  }

  // fill lane info
  auto lane_info = env_model_output.mutable_dec_out_laneinfo_bus();
  lane_info->set_dec_out_lanenum_u8(dec_info.ego_info_.valid_lane_size_);
  lane_info->set_dec_out_laneseqfromlft_u8(
      dec_info.ego_info_.lane_idx_from_left_);
  lane_info->set_dec_out_laneseqfromrgt_u8(
      dec_info.ego_info_.lane_idx_from_right_);

  // lane change info
  auto lane_change_info = env_model_output.mutable_dec_out_lanechangeinfo_bus();
  eLMT_LaneMrkrTyp left_lane_type = dec_info.ego_info_.left_boundary_type_;
  eLMT_LaneMrkrTyp right_lane_type = dec_info.ego_info_.right_boundary_type_;

  if (left_lane_type == eLMT_LaneMrkrTyp::LMT_None ||
      left_lane_type == eLMT_LaneMrkrTyp::LMT_Dashed ||
      left_lane_type == eLMT_LaneMrkrTyp::LMT_DoubleDashedLine ||
      left_lane_type == eLMT_LaneMrkrTyp::LMT_LeftSolidRightDashed) {
    lane_change_info->set_dec_out_d_lftlinetypchgpntneardist_sg(
        dec_info.ego_info_.left_boundary_dash2solid_dist_);
    AINFO << "left: dash";
  } else {
    lane_change_info->set_dec_out_d_lftlinetypchgpntneardist_sg(
        dec_info.ego_info_.left_lc_start_dist_);
    AINFO << "left: solid";
  }
  if (right_lane_type == eLMT_LaneMrkrTyp::LMT_None ||
      right_lane_type == eLMT_LaneMrkrTyp::LMT_Dashed ||
      right_lane_type == eLMT_LaneMrkrTyp::LMT_DoubleDashedLine ||
      right_lane_type == eLMT_LaneMrkrTyp::LMT_RightSolidLeftDashed) {
    lane_change_info->set_dec_out_d_rgtlinetypchgpntneardist_sg(
        dec_info.ego_info_.right_boundary_dash2solid_dist_);
    AINFO << "right: dash";
  } else {
    lane_change_info->set_dec_out_d_rgtlinetypchgpntneardist_sg(
        dec_info.ego_info_.right_lc_start_dist_);
    AINFO << "right: solid";
  }
  AINFO << " -----------line type--------------";
  AINFO << "left: "
        << lane_change_info->dec_out_d_lftlinetypchgpntneardist_sg();
  AINFO << "right: "
        << lane_change_info->dec_out_d_rgtlinetypchgpntneardist_sg();

  // fill force lane change request result
  auto force_lc_result =
      env_model_output.mutable_dec_out_forcelanechgrslt_bus();
  force_lc_result->set_flcr_stat_forcelcddirt_elcrd(
      dec_info.force_lc_.force_lc_direct_);
  force_lc_result->set_flcr_stat_forcelcrsn_elcrr(
      dec_info.force_lc_.force_lc_reason_);
  force_lc_result->set_flcr_is_approach_ramp_bl(
      dec_info.force_lc_.is_approach_ramp);
  force_lc_result->set_flcr_is_inhibit_lftlc_bl(
      dec_info.inhibit_lc_.inhibit_left_lane_change_);
  force_lc_result->set_flcr_is_inhibit_rhtlc_bl(
      dec_info.inhibit_lc_.inhibit_right_lane_change_);
  // fill noa_line turn light direction
  if (noa_line.is_trigger) {
    if (noa_line.direction == -1) {
      force_lc_result->set_dec_out_stat_turnlightreq_etlr(
          zark::ads_common::eTLR_TurnLightType::TLR_Right_LC);
    } else if (noa_line.direction == 1) {
      force_lc_result->set_dec_out_stat_turnlightreq_etlr(
          zark::ads_common::eTLR_TurnLightType::TLR_Left_LC);
    }
  }
}

void ForceLaneChangeDecision::FillTakeOverReqInfo(
    const ForceLcMeasureInfo &dec_info, DEC_Outputs &env_model_output) {
  auto force_lc_result =
      env_model_output.mutable_dec_out_forcelanechgrslt_bus();
  force_lc_result->set_flcr_stat_takeoverreq_etor(
      dec_info.take_over_.take_over_request_);
  force_lc_result->set_flcr_stat_takeoverreqrsn_etorr(
      dec_info.take_over_.take_over_req_reason_);
}

void ForceLaneChangeDecision::Reset() {
  last_other_ramp_.AllReset();
  last_req_keep_.Reset();
  env_model_output_.Clear();
}

std::string ForceLaneChangeDecision::Name() const {
  return "ForceLaneChangeDecision";
}

}  // namespace planning
}  // namespace zark
