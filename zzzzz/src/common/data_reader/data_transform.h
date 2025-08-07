/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file data_transform.h
 **/

#ifndef DATA_TRANSFORM_H
#define DATA_TRANSFORM_H

#include "apps/planning/src/common/data_reader/forcelc_measure.h"
#include "apps/planning/src/common/data_reader/road_section_info.h"
#include "messages/localization/localization.pb.h"
#include "messages/map_service/all_map_new.pb.h"
#include "messages/mapfusion/map_fusion.pb.h"
#include "messages/planning/ads_decision_measure.pb.h"

using namespace zark::ads_decision;

namespace zark {
namespace planning {

inline void TransformLane2Proto(
    const LaneTopoInfo& inner_lane,
    zark::ads_decision::LaneTopoInfo& proto_lane_topo_info) {
  proto_lane_topo_info.mutable_lane_topo()->CopyFrom(inner_lane.lane_topo);
  proto_lane_topo_info.set_change_direction(inner_lane.change_direction);
  proto_lane_topo_info.set_is_recommend(inner_lane.is_recommend);
  proto_lane_topo_info.set_forward_is_recommend(
      inner_lane.forward_is_recommend);
  proto_lane_topo_info.set_remain_distance(inner_lane.remain_distance);
  proto_lane_topo_info.set_non_recommend_distance(
      inner_lane.non_recommend_distance);
  proto_lane_topo_info.set_lane_sequence(inner_lane.lane_sequence);
  proto_lane_topo_info.set_change_count(inner_lane.change_count);
  proto_lane_topo_info.set_speed_limit(inner_lane.speed_limit);
  proto_lane_topo_info.set_final_id(inner_lane.final_id);
}

inline void TransformSection2Proto(
    const NavSection& inner_section,
    zark::ads_decision::NavSection& proto_section) {
  proto_section.mutable_lanes()->Clear();
  for (const auto& lane : inner_section.lanes) {
    zark::ads_decision::LaneTopoInfo proto_lane_topo_info;
    TransformLane2Proto(lane.second, proto_lane_topo_info);
    proto_section.add_lanes()->CopyFrom(proto_lane_topo_info);
  }
  proto_section.mutable_topo_range()->set_first_successor_seq(
      inner_section.topo_range.first_successor_seq);
  proto_section.mutable_topo_range()->set_last_successor_seq(
      inner_section.topo_range.last_successor_seq);
  proto_section.mutable_topo_range()
      ->mutable_priority_sucessor_range()
      ->Clear();
  for (const auto& priority_sucessor_range :
       inner_section.topo_range.priority_sucessor_range) {
    zark::ads_decision::PriorityTop proto_priority_top;
    proto_priority_top.set_current_seq(priority_sucessor_range.first);
    proto_priority_top.set_forward_lane(priority_sucessor_range.second);

    proto_section.mutable_topo_range()->add_priority_sucessor_range()->CopyFrom(
        proto_priority_top);
  }

  proto_section.set_start_s(inner_section.start_s);
  proto_section.set_end_s(inner_section.end_s);
  proto_section.set_valid_lane_size(inner_section.valid_lane_size);
  proto_section.set_section_id(inner_section.section_id);
  proto_section.set_all_lane_connect(inner_section.all_lane_connect);
  proto_section.set_all_transition_continue(
      inner_section.all_transition_continue);
  proto_section.set_next_new_road(inner_section.next_new_road);
  proto_section.set_min_speed_limit(inner_section.min_spee_limit);
  proto_section.set_max_speed_limit(inner_section.max_speed_limit);
}

inline void TransformMeas2Proto(const NavigationInfo* refline_navi,
                                const ForceLcMeasureInfo& force_result_meas,
                                DecisionMeasInfo& output_result) {
  // process RefLineNaviInfo
  zark::ads_decision::RefLineNaviInfo proto_ref_line_navi_info;
  proto_ref_line_navi_info.mutable_sections()->Clear();
  if (refline_navi) {
    for (const auto& section : refline_navi->sections) {
      zark::ads_decision::NavSection proto_nav_section;
      TransformSection2Proto(section, proto_nav_section);
      proto_ref_line_navi_info.add_sections()->CopyFrom(proto_nav_section);
    }

    proto_ref_line_navi_info.mutable_change_points()->Clear();
    for (const auto& point : refline_navi->change_points) {
      proto_ref_line_navi_info.add_change_points()->CopyFrom(point);
    }

    proto_ref_line_navi_info.set_current_section_id(
        refline_navi->current_section_id);
    proto_ref_line_navi_info.set_current_lane_id(refline_navi->current_lane_id);
    proto_ref_line_navi_info.set_currrent_lane_pose(
        refline_navi->currrent_lane_pose);

    proto_ref_line_navi_info.mutable_noa_line()->set_is_trigger(
        refline_navi->noa_line.is_trigger);
    switch (refline_navi->noa_line.direction) {
      case -1:
        proto_ref_line_navi_info.mutable_noa_line()->set_direction(
            zark::ads_common::LCRD_Right_LC);
        break;
      case 0:
        proto_ref_line_navi_info.mutable_noa_line()->set_direction(
            zark::ads_common::LCRD_None_LC);
        break;
      case 1:
        proto_ref_line_navi_info.mutable_noa_line()->set_direction(
            zark::ads_common::LCRD_Left_LC);
        break;
      default:
        proto_ref_line_navi_info.mutable_noa_line()->set_direction(
            zark::ads_common::LCRD_None_LC);
        break;
    }
    proto_ref_line_navi_info.set_valid_info(refline_navi->valid_info);

    auto proto_cur_lane = proto_ref_line_navi_info.mutable_current_lane_info();
    const auto& inner_lane_info = refline_navi->current_lane_info;
    TransformLane2Proto(inner_lane_info, *proto_cur_lane);

    proto_ref_line_navi_info.set_ego_speed(refline_navi->ego_speed);

    output_result.mutable_refline_navi_info()->CopyFrom(
        proto_ref_line_navi_info);
  }
  // Done RefLineNaviInfo

  // process ForceLcMeasureInfo
  zark::ads_decision::ForceLcResult proto_force_lc_result;
  proto_force_lc_result.mutable_force_lc_info()->set_time_now(
      force_result_meas.force_lc_.time_now_);
  proto_force_lc_result.mutable_force_lc_info()->set_change_count(
      force_result_meas.ego_info_.change_count_);
  proto_force_lc_result.mutable_force_lc_info()->set_non_recommend_distance_(
      force_result_meas.ego_info_.non_recommend_distance_);
  proto_force_lc_result.mutable_force_lc_info()->set_dist_lc_senario(
      force_result_meas.force_lc_.dist_lc_senario_);
  proto_force_lc_result.mutable_force_lc_info()->set_dist_general_lc_gate(
      force_result_meas.force_lc_.dist_general_lc_gate_);
  proto_force_lc_result.mutable_force_lc_info()->set_dist_jam_lc_gate(
      force_result_meas.force_lc_.dist_jam_lc_gate_);
  proto_force_lc_result.mutable_force_lc_info()->set_dist_tunnel_lc_gate(
      force_result_meas.force_lc_.dist_tunnel_lc_gate_);
  proto_force_lc_result.mutable_force_lc_info()->set_dist_lc_gate(
      force_result_meas.force_lc_.dist_lc_gate_);
  proto_force_lc_result.mutable_force_lc_info()->set_ego_speed(
      force_result_meas.ego_info_.ego_speed_);
  proto_force_lc_result.mutable_force_lc_info()->set_speed_limit(
      force_result_meas.ego_info_.speed_limit_);
  proto_force_lc_result.mutable_force_lc_info()->set_force_lc_reason(
      force_result_meas.force_lc_.force_lc_reason_);
  proto_force_lc_result.mutable_force_lc_info()->set_force_lc_direct(
      force_result_meas.force_lc_.force_lc_direct_);
  proto_force_lc_result.mutable_force_lc_info()->set_lane_idx_from_left(
      force_result_meas.ego_info_.lane_idx_from_left_);
  proto_force_lc_result.mutable_force_lc_info()->set_lane_idx_from_right(
      force_result_meas.ego_info_.lane_idx_from_right_);
  proto_force_lc_result.mutable_force_lc_info()->set_valid_lane_size(
      force_result_meas.ego_info_.valid_lane_size_);

  proto_force_lc_result.mutable_off_ramp_info()->set_is_off_ramp(
      force_result_meas.off_ramp_.is_off_ramp_);
  proto_force_lc_result.mutable_off_ramp_info()->set_dist_off_ramp_start(
      force_result_meas.off_ramp_.dist_off_ramp_start_);
  proto_force_lc_result.mutable_off_ramp_info()->set_dist_off_ramp_end(
      force_result_meas.off_ramp_.dist_off_ramp_end_);
  proto_force_lc_result.mutable_off_ramp_info()->set_offramp_direct(
      force_result_meas.off_ramp_.offramp_direct_);

  proto_force_lc_result.mutable_on_ramp_info()->set_is_on_ramp(
      force_result_meas.on_ramp_.is_on_ramp_);
  proto_force_lc_result.mutable_on_ramp_info()->set_dist_on_ramp_start(
      force_result_meas.on_ramp_.dist_on_ramp_start_);
  proto_force_lc_result.mutable_on_ramp_info()->set_dist_on_ramp_end(
      force_result_meas.on_ramp_.dist_on_ramp_end_);
  proto_force_lc_result.mutable_on_ramp_info()->set_onramp_direct(
      force_result_meas.on_ramp_.onramp_direct_);

  proto_force_lc_result.mutable_avoid_merge_info()->set_is_avoid_traffic_flow(
      force_result_meas.avoid_merge_.is_avoid_traffic_flow_);
  proto_force_lc_result.mutable_avoid_merge_info()
      ->set_dist_avoid_traffic_flow_start(
          force_result_meas.avoid_merge_.dist_avoid_traffic_flow_start_);
  proto_force_lc_result.mutable_avoid_merge_info()
      ->set_dist_avoid_traffic_flow_end(
          force_result_meas.avoid_merge_.dist_avoid_traffic_flow_end_);
  proto_force_lc_result.mutable_avoid_merge_info()->set_avoid_merge_lane_direct(
      force_result_meas.avoid_merge_.avoid_merge_lane_direct_);

  proto_force_lc_result.mutable_other_scenario_info()->set_has_tunnel(
      force_result_meas.other_scenario_.has_tunnel_);
  proto_force_lc_result.mutable_other_scenario_info()->set_has_traffic_jam(
      force_result_meas.other_scenario_.has_traffic_jam_);
  proto_force_lc_result.mutable_other_scenario_info()->set_has_road_construct(
      force_result_meas.other_scenario_.has_road_construct_);
  proto_force_lc_result.mutable_other_scenario_info()->set_has_toll_station(
      force_result_meas.other_scenario_.has_toll_station_);
  proto_force_lc_result.mutable_other_scenario_info()
      ->set_dist_toll_station_start(
          force_result_meas.other_scenario_.dist_toll_station_start_);

  proto_force_lc_result.mutable_take_over_info()->set_take_over_request(
      force_result_meas.take_over_.take_over_request_);
  proto_force_lc_result.mutable_take_over_info()->set_take_over_req_reason(
      force_result_meas.take_over_.take_over_req_reason_);
  proto_force_lc_result.mutable_take_over_info()->set_remain_distance_(
      force_result_meas.take_over_.dist_remain_);

  output_result.mutable_force_lc_result()->CopyFrom(proto_force_lc_result);
  // Done ForceLcMeasureInfo
}

inline void CombineDecOutputsForPnC(
    const std::pair<uint64_t, zark::ads_decision::DEC_Outputs>& ref_line_result,
    zark::ads_decision::DEC_Outputs& out_put_result) {
  uint64_t refline_satrt_time = ref_line_result.first;
  const DEC_Outputs& fitted_line_result = ref_line_result.second;
  const uint32_t merge_scenario =
      out_put_result.dec_out_noarampline_bus().reserved();
  ads_common::eLMT_LaneMrkrTyp left_boudary_type =
      out_put_result.dec_out_hostlnelftline_bus().lli_stat_linetype_elmt();
  ads_common::eLMT_LaneMrkrTyp right_boudary_type =
      out_put_result.dec_out_hostlnergtline_bus().lli_stat_linetype_elmt();

  out_put_result.mutable_dec_out_hostlnerearcentline_bus()->CopyFrom(
      fitted_line_result.dec_out_hostlnerearcentline_bus());
  out_put_result.mutable_dec_out_hostlnelftline_bus()->CopyFrom(
      fitted_line_result.dec_out_hostlnelftline_bus());
  out_put_result.mutable_dec_out_hostlnergtline_bus()->CopyFrom(
      fitted_line_result.dec_out_hostlnergtline_bus());

  out_put_result.mutable_dec_out_hostlnelftline_bus()
      ->set_lli_stat_linetype_elmt(left_boudary_type);
  out_put_result.mutable_dec_out_hostlnergtline_bus()
      ->set_lli_stat_linetype_elmt(right_boudary_type);

  out_put_result.mutable_dec_out_lftlnerearcentline_bus()->CopyFrom(
      fitted_line_result.dec_out_lftlnerearcentline_bus());
  out_put_result.mutable_dec_out_lftlnelftline_bus()->CopyFrom(
      fitted_line_result.dec_out_lftlnelftline_bus());
  out_put_result.mutable_dec_out_lftlnergtline_bus()->CopyFrom(
      fitted_line_result.dec_out_lftlnergtline_bus());

  out_put_result.mutable_dec_out_rgtlnerearcentline_bus()->CopyFrom(
      fitted_line_result.dec_out_rgtlnerearcentline_bus());
  out_put_result.mutable_dec_out_rgtlnelftline_bus()->CopyFrom(
      fitted_line_result.dec_out_rgtlnelftline_bus());
  out_put_result.mutable_dec_out_rgtlnergtline_bus()->CopyFrom(
      fitted_line_result.dec_out_rgtlnergtline_bus());

  out_put_result.mutable_dec_out_lftroadedge_bus()->CopyFrom(
      fitted_line_result.dec_out_lftroadedge_bus());

  out_put_result.mutable_dec_out_rgtroadedge_bus()->CopyFrom(
      fitted_line_result.dec_out_rgtroadedge_bus());

  out_put_result.mutable_dec_out_noarampline_bus()->CopyFrom(
      fitted_line_result.dec_out_noarampline_bus());

  out_put_result.mutable_dec_out_noarampline_bus()->set_reserved(
      merge_scenario);
  out_put_result.mutable_dec_out_lanechangeinfo_bus()
      ->set_dec_out_is_hostlnechgtolft_bl(
          fitted_line_result.dec_out_lanechangeinfo_bus()
              .dec_out_is_hostlnechgtolft_bl());
  out_put_result.mutable_dec_out_lanechangeinfo_bus()
      ->set_dec_out_is_hostlnechgtorgt_bl(
          fitted_line_result.dec_out_lanechangeinfo_bus()
              .dec_out_is_hostlnechgtorgt_bl());
  out_put_result.set_dec_out_t_decexcttime_sg(refline_satrt_time);
  out_put_result.mutable_dec_out_curv_path_bus()->CopyFrom(
      fitted_line_result.dec_out_curv_path_bus());
}

inline void EmptyMsgCheck(zark::ads_decision::DEC_Outputs& out_msg) {
  // const int8_t kPathNodeNum = 5;
  if (!out_msg.has_dec_out_v_hostlnespdlmt_bus()) {
    out_msg.mutable_dec_out_v_hostlnespdlmt_bus();
  }
  if (!out_msg.dec_out_v_hostlnespdlmt_bus().has_dec_out_v_spdlmt1_bus()) {
    out_msg.mutable_dec_out_v_hostlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt1_bus();
  }
  if (!out_msg.dec_out_v_hostlnespdlmt_bus().has_dec_out_v_spdlmt2_bus()) {
    out_msg.mutable_dec_out_v_hostlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt2_bus();
  }
  if (!out_msg.dec_out_v_hostlnespdlmt_bus().has_dec_out_v_spdlmt3_bus()) {
    out_msg.mutable_dec_out_v_hostlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt3_bus();
  }
  if (!out_msg.dec_out_v_hostlnespdlmt_bus().has_dec_out_v_spdlmt4_bus()) {
    out_msg.mutable_dec_out_v_hostlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt4_bus();
  }
  if (!out_msg.dec_out_v_hostlnespdlmt_bus().has_dec_out_v_spdlmt5_bus()) {
    out_msg.mutable_dec_out_v_hostlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt5_bus();
  }

  if (!out_msg.has_dec_out_v_lftlnespdlmt_bus()) {
    out_msg.mutable_dec_out_v_lftlnespdlmt_bus();
  }
  if (!out_msg.dec_out_v_lftlnespdlmt_bus().has_dec_out_v_spdlmt1_bus()) {
    out_msg.mutable_dec_out_v_lftlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt1_bus();
  }
  if (!out_msg.dec_out_v_lftlnespdlmt_bus().has_dec_out_v_spdlmt2_bus()) {
    out_msg.mutable_dec_out_v_lftlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt2_bus();
  }
  if (!out_msg.dec_out_v_lftlnespdlmt_bus().has_dec_out_v_spdlmt3_bus()) {
    out_msg.mutable_dec_out_v_lftlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt3_bus();
  }
  if (!out_msg.dec_out_v_lftlnespdlmt_bus().has_dec_out_v_spdlmt4_bus()) {
    out_msg.mutable_dec_out_v_lftlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt4_bus();
  }
  if (!out_msg.dec_out_v_lftlnespdlmt_bus().has_dec_out_v_spdlmt5_bus()) {
    out_msg.mutable_dec_out_v_lftlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt5_bus();
  }

  if (!out_msg.has_dec_out_v_rgtlnespdlmt_bus()) {
    out_msg.mutable_dec_out_v_rgtlnespdlmt_bus();
  }
  if (!out_msg.dec_out_v_rgtlnespdlmt_bus().has_dec_out_v_spdlmt1_bus()) {
    out_msg.mutable_dec_out_v_rgtlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt1_bus();
  }
  if (!out_msg.dec_out_v_rgtlnespdlmt_bus().has_dec_out_v_spdlmt2_bus()) {
    out_msg.mutable_dec_out_v_rgtlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt2_bus();
  }
  if (!out_msg.dec_out_v_rgtlnespdlmt_bus().has_dec_out_v_spdlmt3_bus()) {
    out_msg.mutable_dec_out_v_rgtlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt3_bus();
  }
  if (!out_msg.dec_out_v_rgtlnespdlmt_bus().has_dec_out_v_spdlmt4_bus()) {
    out_msg.mutable_dec_out_v_rgtlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt4_bus();
  }
  if (!out_msg.dec_out_v_rgtlnespdlmt_bus().has_dec_out_v_spdlmt5_bus()) {
    out_msg.mutable_dec_out_v_rgtlnespdlmt_bus()
        ->mutable_dec_out_v_spdlmt5_bus();
  }

  if (!out_msg.has_dec_out_forcelanechgrslt_bus()) {
    out_msg.mutable_dec_out_forcelanechgrslt_bus();
  }

  if (!out_msg.has_dec_out_v_electreyespdlmt_bus()) {
    out_msg.mutable_dec_out_v_electreyespdlmt_bus();
  }
  if (!out_msg.has_dec_out_v_zonespdlmt_bus()) {
    out_msg.mutable_dec_out_v_zonespdlmt_bus();
  }

  if (!out_msg.has_dec_out_curv_path_bus() ||
      !out_msg.dec_out_curv_path_bus().has_curv_nodes1_bus()) {
    auto path = out_msg.mutable_dec_out_curv_path_bus();
    path->mutable_curv_nodes1_bus();
    path->mutable_curv_nodes2_bus();
    path->mutable_curv_nodes3_bus();
    path->mutable_curv_nodes4_bus();
    path->mutable_curv_nodes5_bus();
    // for (uint8_t i = 0; i < kPathNodeNum; i++) {
    //   path->add_curv_nodes();
    // }
  }

  if (!out_msg.has_dec_out_slope_path_bus() ||
      out_msg.dec_out_slope_path_bus().has_slope_nodes1_bus()) {
    auto path = out_msg.mutable_dec_out_slope_path_bus();
    path->mutable_slope_nodes1_bus();
    path->mutable_slope_nodes2_bus();
    path->mutable_slope_nodes3_bus();
    path->mutable_slope_nodes4_bus();
    path->mutable_slope_nodes5_bus();
    // for (uint8_t i = 0; i < kPathNodeNum; i++) {
    //   path->mutable_slope_nodes1_bus();
    // }
  }

  if (!out_msg.has_dec_out_tunnel_bus()) {
    out_msg.mutable_dec_out_tunnel_bus();
  }

  if (!out_msg.has_dec_out_hostlnerearcentline_bus()) {
    out_msg.mutable_dec_out_hostlnerearcentline_bus();
  }
  if (!out_msg.has_dec_out_hostlnelftline_bus()) {
    out_msg.mutable_dec_out_hostlnelftline_bus();
  }
  if (!out_msg.has_dec_out_hostlnergtline_bus()) {
    out_msg.mutable_dec_out_hostlnergtline_bus();
  }

  if (!out_msg.has_dec_out_lftlnerearcentline_bus()) {
    out_msg.mutable_dec_out_lftlnerearcentline_bus();
  }
  if (!out_msg.has_dec_out_lftlnelftline_bus()) {
    out_msg.mutable_dec_out_lftlnelftline_bus();
  }
  if (!out_msg.has_dec_out_lftlnergtline_bus()) {
    out_msg.mutable_dec_out_lftlnergtline_bus();
  }

  if (!out_msg.has_dec_out_rgtlnerearcentline_bus()) {
    out_msg.mutable_dec_out_rgtlnerearcentline_bus();
  }
  if (!out_msg.has_dec_out_rgtlnelftline_bus()) {
    out_msg.mutable_dec_out_rgtlnelftline_bus();
  }
  if (!out_msg.has_dec_out_rgtlnergtline_bus()) {
    out_msg.mutable_dec_out_rgtlnergtline_bus();
  }

  if (!out_msg.has_dec_out_lftroadedge_bus()) {
    out_msg.mutable_dec_out_lftroadedge_bus();
  }
  if (!out_msg.has_dec_out_rgtroadedge_bus()) {
    out_msg.mutable_dec_out_rgtroadedge_bus();
  }

  if (!out_msg.has_dec_out_stateinfo_bus()) {
    out_msg.mutable_dec_out_stateinfo_bus();
  }

  if (!out_msg.has_dec_out_distanceinfo_bus()) {
    out_msg.mutable_dec_out_distanceinfo_bus();
  }

  if (!out_msg.has_dec_out_locdeviatinfo_bus()) {
    out_msg.mutable_dec_out_locdeviatinfo_bus();
  }

  if (!out_msg.has_dec_out_rampinfo_bus()) {
    out_msg.mutable_dec_out_rampinfo_bus();
  }

  if (!out_msg.has_dec_out_laneinfo_bus()) {
    out_msg.mutable_dec_out_laneinfo_bus();
  }

  if (!out_msg.has_dec_out_planpropinfo_bus()) {
    out_msg.mutable_dec_out_planpropinfo_bus();
  }
}

inline double SpeedConvert(const uint32_t speed_mps) {
  const double kSpeedBase = 5.0;  //[kph]
  double speed_kph = speed_mps * kSpeedMpsToKph;
  double rounded_speed = static_cast<double>(
      std::floor(speed_kph / kSpeedBase + 0.5) * kSpeedBase);
  return rounded_speed;
}

}  // namespace planning
}  // namespace zark
#endif  // end of DATA_TRANSFORM_H
