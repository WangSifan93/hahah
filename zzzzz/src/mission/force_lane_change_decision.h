/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file force_lane_change_decision.h
 **/

#pragma once

#include <list>

#include "apps/planning/src/common/clock.h"
#include "apps/planning/src/common/data_reader/data_reader.h"
#include "apps/planning/src/common/data_reader/forcelc_measure.h"
#include "apps/planning/src/common/data_reader/road_section_info.h"
#include "apps/planning/src/common/log.h"
#include "apps/planning/src/common/mission.h"
#include "messages/planning/ads_decision.pb.h"

namespace zark {
namespace planning {
constexpr uint8_t kMaxTriggerCnt = 2;  // really count is kMaxTriggerCnt +1
constexpr uint8_t kMaxCancelCnt = 20;

constexpr float Kcenti2meter = 0.01f;
class ForceLaneChangeDecision {
 public:
  ForceLaneChangeDecision() = default;

  /**
   * @struct LastNonTargetRamp
   * @brief Structure to record information about the last non-target ramp.
   */
  struct LastNonTargetRamp {
   public:
    LastNonTargetRamp()
        : keep_inhibit_(false),
          finish_inhibit_(false),
          keep_inhibit_distance_(0.0),
          need_inhibit_dir_(0),
          last_update_stamp_(0.0) {}

    void Reset() {
      keep_inhibit_ = false;
      finish_inhibit_ = false;
      keep_inhibit_distance_ = 0.0;
      last_update_stamp_ = zark::common::Clock::NowInSeconds();
    }
    void AllReset() {
      Reset();
      need_inhibit_dir_ = 0;
    }

    bool keep_inhibit_;            // start keep inhibit if true
    bool finish_inhibit_;          // finish keep inhibit if true
    float keep_inhibit_distance_;  // keep inhibit until distance > param
    int8_t need_inhibit_dir_;      // 0 no inhibit,1 left, 2 right, 3 all.
    double last_update_stamp_;     // last update keep_inhibit_distance_ stamp
  };

  struct OnRampDistKeep {
   public:
    OnRampDistKeep()
        : keep_onramp_(false),
          has_off_ramp_(false),
          keep_ramp_distance_(0.0),
          last_time_stamp_(0.0) {}

    void Reset() {
      keep_onramp_ = false;
      has_off_ramp_ = false;
      keep_ramp_distance_ = 0.0;
      last_time_stamp_ = 0.0;
    }

    bool keep_onramp_;          // keep record on ramp
    bool has_off_ramp_;         // close to off ramp
    float keep_ramp_distance_;  // keep calculate distance from onramp start
    double last_time_stamp_;    // last update time stamp
  };

  bool DeciderProc(const NavigationInfo &navi_info,
                   const DEC_Outputs &fitted_result);

  std::string Name() const;

  void GetForceChangeLaneResult(const NavigationInfo &navi_info,
                                ForceLcMeasureInfo &dec_info);
  void GetTakeOverResult(ForceLcMeasureInfo &dec_info);
  const DEC_Outputs GetEnvModelOutput() const { return env_model_output_; }
  const ForceLcMeasureInfo GetForceLcMeasureInfo() const {
    return decision_info_;
  }
  const ads_common::eLMT_LaneMrkrTyp GetLeftBoundaryType() {
    return decision_info_.ego_info_.left_boundary_type_;
  }
  const ads_common::eLMT_LaneMrkrTyp GetRightBoundaryType() {
    return decision_info_.ego_info_.right_boundary_type_;
  }

  void ResetMeasureRequest();
  void Reset();

  /**
   * @brief Structure to store information related to the last lane change
   * request.
   */
  struct LastReqKeepInfo {
    eLCRD_ForceLaneChgReqDir last_direction =
        eLCRD_ForceLaneChgReqDir::LCRD_None_LC; /**< Last direction of the
                                                   lane change request. */
    eLCRR_ForceLaneChgReqRsn last_reason =
        eLCRR_ForceLaneChgReqRsn::LCRR_NoReason; /**< Last reason for the lane
                                                    change request. */
    uint8_t force_trigger_cnt =
        0; /**< Count of force lane change request triggers. */
    uint8_t force_cancel_cnt =
        0; /**< Count of force lane change request cancellations. */

    /**
     * @brief Reset the LastReqKeepInfo structure members to default values.
     *
     * This function resets the last_direction, last_reason,
     * force_trigger_cnt, and force_cancel_cnt members of the LastReqKeepInfo
     * structure to their default values.
     */
    void Reset() {
      last_direction = eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
      last_reason = eLCRR_ForceLaneChgReqRsn::LCRR_NoReason;
      force_trigger_cnt = 0;
      force_cancel_cnt = 0;
    }
  };

  /**
   * @brief struct to keep last route distance information
   *
   */
  struct LastRouteInfo {
   public:
    LastRouteInfo() : sd_distance(0.0f), hd_distance(0.0f) {}

    void Reset() {
      sd_distance = 0.0f;
      hd_distance = 0.0f;
    }
    float sd_distance;
    float hd_distance;
  };

  const LastReqKeepInfo GetLastKeepInfo() { return last_req_keep_; }

  const ForceLcMeasureInfo GetForceMeasInfo() { return decision_info_; }

 private:
  void ScenarioProc(const OffRampStr &off_ramp, const OnRampStr &on_ramp,
                    const AvoidMergeStr &avoid_merge,
                    const EgoInfoStr &ego_info, const NavigationInfo &navi_info,
                    ForceLcReqStr &force_lc, OnRampDistKeep &last_onramp);

  /**
   * @brief Based on the road segments of map, four sections of speed
   * limit information are extracted for noa_pnc. The first speed section is
   * the minimum speed limit within 100m from ego, and different processing
   * needs to be carried out according to whether the remaining length of ego
   * current road segment is sufficient for 100m. The following three sections
   * are issued according to the map data.
   * @param road_segments the all road segmetns of ehr map.
   * @param current_seg_index the index of ego current road segment.
   */
  void UpdateLineInfo(const std::list<NavSection> &road_segments,
                      const std::list<NavSection>::const_iterator &current_iter,
                      const uint8_t current_lane_seq,
                      const ForceLcMeasureInfo &dec_info);
  void ExtendSpeedLimits(
      const std::list<NavSection> &road_segments,
      const std::list<NavSection>::const_iterator &current_seg,
      const float &fix_dis, const float &accumulate_dis, DEC_Outputs &output);
  void FillSpeedLimitsInfo(const uint8_t valid_lane_size,
                           const uint8_t current_lane_seq,
                           DEC_Outputs &env_model_output);
  void ExtractSectionInfo(
      const google::protobuf::RepeatedPtrField<zark::hdmap_new::SectionInfo>
          &road_segments,
      const NavigationInfo &navi_info, ForceLcMeasureInfo &dec_info);
  void UpdateLeftBoundaryInfo(
      const std::unordered_map<std::string, hdmap_new::LaneInfo> lanes_topo,
      const std::list<NavSection> &road_segments,
      const std::string current_lane_id, ForceLcMeasureInfo &dec_info);
  void UpdateRightBoundaryInfo(
      const std::unordered_map<std::string, hdmap_new::LaneInfo> lanes_topo,
      const std::list<NavSection> &road_segments,
      const std::string current_lane_id, ForceLcMeasureInfo &dec_info);
  void UpdateMapInfo(const std::vector<hdmap_new::ChangePoint> change_points,
                     ForceLcMeasureInfo &dec_info);
  void CalcuFullConnectRemainDistance(
      const std::unordered_map<std::string, zark::planning::LaneTopoInfo>
          cur_lanes,
      EgoInfoStr &ego_info);
  // void UpdateEgoInfo(const SectionInfo &mapf_output, EgoInfoStr
  // &ego_info);
  void UpdateEnvInfo(const NavigationInfo &navi_info,
                     ForceLcMeasureInfo &dec_info,
                     LastRouteInfo &last_route_info);
  // int8_t SwitchLanePosn(Lane_Rela_Pos lane_posn);
  void ScenarioOffRampProc(const OffRampStr &off_ramp,
                           const EgoInfoStr &ego_info,
                           ForceLcReqStr &force_lc_req, TakeOverStr &take_over);
  void ScenarioOnRampProc(const OnRampStr &on_ramp, ForceLcReqStr &force_lc_req,
                          TakeOverStr &take_over);
  //   void ScenarioAvoidMergTrafficFlowProc(const PlanMsg_EgoPosition &ego_pos,
  //                                         const OffRampStr &off_ramp,
  //                                         const AvoidMergeStr &avoid_merge,
  //                                         ForceLcReqStr &force_lc_req);
  // void ScenarioOtherSplitRampProc(const OffRampStr &other_split_ramp,
  //                                 ForceLcReqStr &force_lc_req);
  void LaneChgParaCacul(ForceLcMeasureInfo &dec_info);
  void InhibitProcess(const OnRampDistKeep &last_onramp,
                      ForceLcMeasureInfo &dec_info);
  float CalcuInhibitDistanceGate(uint8_t lane_change_count, float vehicle_speed,
                                 float speed_limit);
  // void InhibitTargetRamp(ForceLcMeasureInfo &dec_info, int8_t &inhibit_dir);
  // void InhibitOtherRamp(ForceLcMeasureInfo &dec_info,
  //                       LastNonTargetRamp &last_other_ramp,
  //                       int8_t &inhibit_dir);

  void FillForceLaneChgReqInfo(const ForceLcMeasureInfo &dec_info,
                               const NoaLine noa_line,
                               const DEC_Outputs &fitted_result,
                               eLCRD_ForceLaneChgReqDir merge_dir,
                               DEC_Outputs &env_model_output);
  void FillTakeOverReqInfo(const ForceLcMeasureInfo &dec_info,
                           DEC_Outputs &env_model_output);
  //   void ExtractOffRmapInfo(const PlanMsg_RampSplit *rampsplits,
  //                           const uint8 splits_size, OffRampStr &off_ramp,
  //                           OffRampStr &other_split_ramp,
  //                           OnRampDistKeep &last_onramp);
  //   void ExtractOnRmapInfo(const PlanMsg_RampMerge *rampmerges,
  //                          const uint8 merges_size, ForceLcMeasureInfo
  //                          &dec_info, OnRampDistKeep &last_onramp, float
  //                          vehicle_speed, uint8 lane_idx_from_right);
  void ExtractOtherInfo(const zark::hdmap_new::SectionInfo &map_info,
                        OtherScenarioStr &other_scen_info);
  int8_t CalcuLaneChgNum(const ForceLcMeasureInfo &dec_info);
  float CalcuGeneralLCDistGate(const ForceLcMeasureInfo &dec_info,
                               const int8_t lane_change_num);
  float CalcuTunnelLCDistGate(const ForceLcMeasureInfo &dec_info,
                              const zark::hdmap_new::SectionInfo &map_info);
  float CalcuTraffJamLCDistGate(const ForceLcMeasureInfo &dec_info,
                                const zark::hdmap_new::SectionInfo &map_info);
  //   bool IsLocalizationLowConf(const ForceLcReqStr &force_lc,
  //                              const PlanMsg_EgoPosition &ego_pos);
  //   void UpdateRemainDist(const PlanMsg_EgoPosition &ego_pos,
  //                         ForceLcMeasureInfo &dec_info);
  void FillEgoPosInfo(const zark::hdmap_new::SectionInfo &map,
                      DEC_Outputs &env_model_output);

  /**
   * @brief Check for continuing lane change requests based on current lane
   * change result output and update last request information.
   *
   * This function performs a continuous frame check to determine whether a
   * request should be sent or canceled based on the current lane result and
   * the information about the last lane change request.
   *
   * @param env_model_output The current lane change result struct
   * @param last_keep_info Information about the last lane change request to
   * consider for continuation.
   */
  void ContinueReqCheck(DEC_Outputs &env_model_output,
                        LastReqKeepInfo &last_keep_info);

  eLCRD_ForceLaneChgReqDir FindMergeLaneChangeDir(
      const NavigationInfo &navi_info);

  void IdentityMergeScenario(
      const DEC_Outputs &fitted_result,
      const std::vector<zark::hdmap_new::ChangePoint> &change_points,
      DEC_Outputs &dec_result);

  ads_common::eLMT_LaneMrkrTyp FillBoundaryType(
      const hdmap_new::LaneBoundaryAttribute_Type &boundary_type);

  void IsNearRampCheck(
      const NavigationInfo &navi_info,
      const std::unordered_map<std::string, zark::planning::LaneTopoInfo>
          cur_lanes,
      ForceLcMeasureInfo &dec_info);

 private:
  // output message
  DEC_Outputs env_model_output_;
  ForceLcMeasureInfo decision_info_;
  constexpr static float Epsilon = 1e-10;
  constexpr static float kph2mps = 0.278;

  LastNonTargetRamp last_other_ramp_;
  OnRampDistKeep last_onramp_;
  LastReqKeepInfo last_req_keep_;
  LastRouteInfo last_route_info_;
};

}  // namespace planning
}  // namespace zark
