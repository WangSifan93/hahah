/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *
 *****************************************************************************/

#include "apps/planning/src/common/proto_convertor/proto_convertor.h"

#include "apps/planning/src/common/log.h"
#include "apps/planning/src/common/math/mpc/utility.h"

constexpr int KNumRollCounter = 15;
namespace zark {
namespace planning {
namespace common {

void ProtoConvertor::RecordPlanningDebugInfo(
    const std::vector<CorridorInfo>& corridor_infos,
    const std::vector<Proposal>& proposals,
    const CorridorInfo* corridor_info_prev, const Mission& mission,
    const LatencyStats& latency_stats, const bool is_replan,
    const std::string& replan_reason,
    PlanningDebugInfo& planning_debug_info_proto) {
  RecordLatencyStatsProto(latency_stats, planning_debug_info_proto);
  RecordMissionProto(mission, planning_debug_info_proto);
  RecordCorridorInfosProto(corridor_infos, planning_debug_info_proto);
  RecordProposalsProto(proposals, planning_debug_info_proto);
  if (corridor_info_prev != nullptr) {
    RecordCorridorInfoProto(
        *corridor_info_prev,
        planning_debug_info_proto.mutable_corridor_info_prev());
  }
  planning_debug_info_proto.set_is_replan(is_replan);
  planning_debug_info_proto.set_replan_reason(replan_reason);
}

void ProtoConvertor::RecordPlanningOutput(
    const zark::planning::ADCTrajectory& trajectory,
    zark::planning::PlanningOutputMsg& planning_output_proto) {
  zark::common::ExternedHeader* header = planning_output_proto.mutable_header();
  RecordHeader(trajectory, header);
  RecordTrajectoryPublished(trajectory, planning_output_proto);
  RecordTrajectoryPolyfit(trajectory, planning_output_proto);
  RecordPlanningStatus(trajectory, planning_output_proto);
}

void ProtoConvertor::RecordLocalRoute(
    const std::list<zark::planning::LocalRoute>& local_routes,
    zark::reference_line_proto::ReferenceLines& ref_lines) {
  uint32_t line_sequen = 1;
  if (local_routes.size() > 0) {
    ref_lines.set_current_line_id(line_sequen);
  }
  ref_lines.set_target_line_id(
      static_cast<uint32_t>(hdmap::RouteSegments::SegmentType::InvalidSegment));
  hdmap::RouteSegments current_segments;
  bool find_current = false;
  for (const auto& local_route : local_routes) {
    zark::reference_line_proto::ReferenceLineInfo* const out_line =
        ref_lines.add_reference_lines();
    const zark::planning::hdmap::Path& map_path = local_route.MapPathInfo();
    const std::vector<zark::planning::hdmap::LaneSegment>& map_lane_segments =
        map_path.lane_segments();
    const hdmap::RouteSegments::SegmentType seg_type =
        local_route.Lanes().GetSegmentType();
    out_line->set_id(static_cast<uint32_t>(seg_type));
    if (seg_type == hdmap::RouteSegments::SegmentType::CurrentSegment) {
      ref_lines.set_host_lc_to_left(local_route.IsHostLC2Left());
      ref_lines.set_host_lc_to_right(local_route.IsHostLC2Right());
    }
    int32_t path_point_index = 0;
    for (const auto& point : local_route.RoutePoints()) {
      RecordLocalRoutePoint(point, map_path, path_point_index, out_line);
      path_point_index++;
    }
    for (const auto& lane_seg : map_lane_segments) {
      RecordLaneSegment(lane_seg, out_line);
    }
    if (local_route.IsTargetRoute() && line_sequen != 1) {
      ref_lines.set_target_line_id(static_cast<uint32_t>(seg_type));
    }
    // TODO(@yfh) navigation lane change info
    line_sequen++;
  }
  if (find_current) {
    // TODO: confirm segments size and active find_current flag;
    zark::reference_line_proto::ReferenceLineInfo* const out_line =
        ref_lines.add_reference_lines();
    out_line->set_id(static_cast<uint32_t>(
        hdmap::RouteSegments::SegmentType::MapCenterLine));
    LocalRoute original_cneter_line(current_segments, LocalRouteConfig());
    for (const auto& point : original_cneter_line.RoutePoints()) {
      zark::reference_line_proto::PathPoint* const out_point =
          out_line->add_path_points();
      auto point_xy = out_point->mutable_base_point();
      point_xy->set_x(point.x());
      point_xy->set_y(point.y());
    }
  }
}

void ProtoConvertor::RecordLatencyStatsProto(
    const LatencyStats& latency_stats,
    PlanningDebugInfo& planning_debug_info_proto) {
  for (const auto& task_latency : latency_stats.task_stats()) {
    auto pb_ptr = planning_debug_info_proto.add_task_latencies();
    pb_ptr->set_task_name(task_latency.name());
    pb_ptr->set_latency(task_latency.time_ms());
  }
}

void ProtoConvertor::RecordMissionProto(
    const Mission& mission, PlanningDebugInfo& planning_debug_info_proto) {
  auto proto_mission = planning_debug_info_proto.mutable_mission();
  switch (mission.lc_request) {
    case Mission::LaneChangeRequest::LANE_KEEP:
      proto_mission->set_lc_request(PlanningDebugInfo::Mission::LANE_KEEP);
      break;
    case Mission::LaneChangeRequest::LC_LEFT:
      proto_mission->set_lc_request(PlanningDebugInfo::Mission::LC_LEFT);
      break;
    case Mission::LaneChangeRequest::LC_RIGHT:
      proto_mission->set_lc_request(PlanningDebugInfo::Mission::LC_RIGHT);
      break;
    default:
      proto_mission->set_lc_request(PlanningDebugInfo::Mission::LANE_KEEP);
      break;
  }

  switch (mission.lc_type) {
    case Mission::LaneChangeType::MANUAL_LC:
      proto_mission->set_lc_type(PlanningDebugInfo::Mission::MANUAL_LC);
      break;
    case Mission::LaneChangeType::ROUTING_LC:
      proto_mission->set_lc_type(PlanningDebugInfo::Mission::ROUTING_LC);
      break;
    case Mission::LaneChangeType::BLOCKING_LC:
      proto_mission->set_lc_type(PlanningDebugInfo::Mission::BLOCKING_LC);
      break;
    case Mission::LaneChangeType::EFFICIENT_LC:
      proto_mission->set_lc_type(PlanningDebugInfo::Mission::EFFICIENT_LC);
      break;
    default:
      proto_mission->set_lc_type(PlanningDebugInfo::Mission::NONE);
      break;
  }

  switch (mission.target_reference_line) {
    case Mission::TargetReflineType::REF_CURRENT_LANE:
      proto_mission->set_target_reference_line(
          PlanningDebugInfo::Mission::REF_CURRENT_LANE);
      break;
    case Mission::TargetReflineType::REF_LEFT_LANE:
      proto_mission->set_target_reference_line(
          PlanningDebugInfo::Mission::REF_LEFT_LANE);
      break;
    case Mission::TargetReflineType::REF_RIGHT_LANE:
      proto_mission->set_target_reference_line(
          PlanningDebugInfo::Mission::REF_RIGHT_LANE);
      break;
    default:
      proto_mission->set_target_reference_line(
          PlanningDebugInfo::Mission::REF_CURRENT_LANE);
      break;
  }

  proto_mission->set_lc_urgency(mission.lc_urgency);
  proto_mission->set_is_near_ramp(mission.is_near_ramp);
  proto_mission->set_is_take_over(mission.is_take_over);
  proto_mission->set_is_lc_ready(mission.is_lc_ready);
  proto_mission->set_is_lc_abort(mission.is_lc_abort);

  auto proto_env_infos = proto_mission->mutable_env_infos();
  proto_env_infos->set_cur_adc_v(mission.env_infos.vehicle_infos.cur_adc_v);
  proto_env_infos->set_driver_set_v(
      mission.env_infos.vehicle_infos.driver_set_v);
  proto_env_infos->set_closer_lane_id(
      mission.env_infos.lane_infos.closer_lane_id);
  proto_env_infos->set_current_lane_width(
      mission.env_infos.lane_infos.current_lane_width);
  proto_env_infos->set_current_s(mission.env_infos.lane_infos.current_sl.s());
  proto_env_infos->set_current_l(mission.env_infos.lane_infos.current_sl.l());

  proto_env_infos->mutable_dis_2_ramp()->Clear();
  for (auto tmp :
       mission.env_infos.lane_infos.route_lc_info.dist_to_next_lc_pt) {
    proto_env_infos->add_dis_2_ramp(tmp);
  }
  proto_env_infos->mutable_ramp_lc_directions()->Clear();
  for (auto tmp : mission.env_infos.lane_infos.route_lc_info.dir_next_lc) {
    proto_env_infos->add_ramp_lc_directions(static_cast<int32_t>(tmp));
  }
  proto_env_infos->mutable_ramp_lc_num()->Clear();
  for (auto tmp :
       mission.env_infos.lane_infos.route_lc_info.num_lc_to_next_lanes) {
    proto_env_infos->add_ramp_lc_num(tmp);
  }

  proto_env_infos->set_is_current_line_blocked(
      mission.env_infos.cur_lane_block_infos.is_current_line_blocked);
  proto_env_infos->set_dis_2_frontstaticobj(
      mission.env_infos.cur_lane_block_infos.dis_2_frontstaticobj);
  proto_env_infos->set_v_cur_lane_limit(
      mission.env_infos.lane_spd_infos.v_cur_lane_limit);
  proto_env_infos->set_v_left_lane_limit(
      mission.env_infos.lane_spd_infos.v_left_lane_limit);
  proto_env_infos->set_v_right_lane_limit(
      mission.env_infos.lane_spd_infos.v_right_lane_limit);
  proto_env_infos->set_v_cur_lane_pass(
      mission.env_infos.lane_spd_infos.v_cur_lane_pass);
  proto_env_infos->set_v_left_lane_pass(
      mission.env_infos.lane_spd_infos.v_left_lane_pass);
  proto_env_infos->set_v_right_lane_pass(
      mission.env_infos.lane_spd_infos.v_right_lane_pass);
  proto_env_infos->set_dis_2_ramp_threshold(
      mission.env_infos.rc_params_infos.dis_2_ramp_threshold);
  proto_env_infos->set_reverse_dis_2_ramp_threshold(
      mission.env_infos.rc_params_infos.reverse_dis_2_ramp_threshold);

  proto_env_infos->set_is_inhibit_left(
      mission.env_infos.inhibit_infos.is_inhibit_left);
  switch (mission.env_infos.inhibit_infos.inhibit_left_type) {
    case InhibitInfos::NONE:
      proto_env_infos->set_inhibit_left_type(0);
      break;
    case InhibitInfos::CUR_LANE_INVALID:
      proto_env_infos->set_inhibit_left_type(1);
      break;
    case InhibitInfos::LANE_TYPE_INVALID:
      proto_env_infos->set_inhibit_left_type(2);
      break;
    case InhibitInfos::CURVATURE_INVALID:
      proto_env_infos->set_inhibit_left_type(3);
      break;
    case InhibitInfos::LEFT_LANE_INVALID:
      proto_env_infos->set_inhibit_left_type(4);
      break;
    default:
      proto_env_infos->set_inhibit_left_type(0);
      break;
  }
  proto_env_infos->set_is_inhibit_right(
      mission.env_infos.inhibit_infos.is_inhibit_right);
  switch (mission.env_infos.inhibit_infos.inhibit_right_type) {
    case InhibitInfos::NONE:
      proto_env_infos->set_inhibit_right_type(0);
      break;
    case InhibitInfos::CUR_LANE_INVALID:
      proto_env_infos->set_inhibit_right_type(1);
      break;
    case InhibitInfos::LANE_TYPE_INVALID:
      proto_env_infos->set_inhibit_right_type(2);
      break;
    case InhibitInfos::CURVATURE_INVALID:
      proto_env_infos->set_inhibit_right_type(3);
      break;
    case InhibitInfos::RIGHT_LANE_INVALID:
      proto_env_infos->set_inhibit_right_type(5);
      break;
    default:
      proto_env_infos->set_inhibit_right_type(0);
      break;
  }
}

void ProtoConvertor::RecordCorridorInfosProto(
    const std::vector<CorridorInfo>& corridor_infos,
    PlanningDebugInfo& planning_debug_info_proto) {
  for (const auto& corridor_info : corridor_infos) {
    auto corridor_info_proto = planning_debug_info_proto.add_corridor_infos();
    RecordCorridorInfoProto(corridor_info, corridor_info_proto);
  }
}

void ProtoConvertor::RecordProposalsProto(
    const std::vector<Proposal>& proposals,
    PlanningDebugInfo& planning_debug_info_proto) {
  for (const auto& proposal : proposals) {
    auto proposal_proto = planning_debug_info_proto.add_proposals();
    RecordProposalProto(proposal, proposal_proto);
  }
}

void ProtoConvertor::RecordCorridorInfoProto(
    const CorridorInfo& corridor_info,
    PlanningDebugInfo_CorridorInfo* corridor_info_proto) {
  corridor_info_proto->set_type(
      ConvertCorridorTypeToString(corridor_info.GetType()));
  RecordSpeedLimitMapProto(corridor_info, corridor_info_proto);
  RecordCorridorPointsProto(corridor_info, corridor_info_proto);
  RecordCorridorSTBoundaryCounterMap(corridor_info, corridor_info_proto);
  RecordSTTopologyProto(corridor_info,
                        *corridor_info_proto->mutable_st_topology());
  RecordCorridorSTProposalsProto(corridor_info, corridor_info_proto);
}

void ProtoConvertor::RecordProposalProto(
    const Proposal& proposal, PlanningDebugInfo_Proposal* proposal_proto) {
  proposal_proto->set_corridor_type(
      ConvertCorridorTypeToString(proposal.GetCorridorInfo()->GetType()));
  RecordSTProposalProto(proposal, *proposal_proto->mutable_st_proposal());
  RecordLonMPCDataProto(proposal, *proposal_proto->mutable_lon_mpc());
  RecordLatMPCDataProto(proposal, *proposal_proto->mutable_lat_mpc());
  RecordEvaluationProto(proposal, *proposal_proto->mutable_evaluation());
  RecordTrajectoryProto(proposal, *proposal_proto->mutable_trajectory());
}

std::string ProtoConvertor::ConvertCorridorTypeToString(
    const CorridorInfo::Type type) {
  switch (type) {
    case CorridorInfo::Type::LANE_KEEP:
      return "LANE_KEEP";
      break;
    case CorridorInfo::Type::LANE_CHANGE:
      return "LANE_CHANGE";
      break;
    case CorridorInfo::Type::STAGING:
      return "STAGING";
      break;
    case CorridorInfo::Type::NUDGE:
      return "NUDGE";
      break;
    default:
      return "INVALID";
  }
}

void ProtoConvertor::RecordSpeedLimitMapProto(
    const CorridorInfo& corridor_info,
    PlanningDebugInfo_CorridorInfo* corridor_info_proto) {
  for (const auto speed_limit : corridor_info.GetSpeedLimitMap()) {
    auto speed_limit_proto = corridor_info_proto->add_speed_limit_map();
    speed_limit_proto->set_name(speed_limit.first);
    for (const auto& point : speed_limit.second.speed_limit_points()) {
      speed_limit_proto->add_s(point.first);
      speed_limit_proto->add_v(point.second);
    }
  }
}

void ProtoConvertor::RecordCorridorPointsProto(
    const CorridorInfo& corridor_info,
    PlanningDebugInfo_CorridorInfo* corridor_info_proto) {
  for (const auto& corridor_point : corridor_info.GetCorridor()) {
    auto corridor_point_trans = corridor_info_proto->add_corridor_points();
    corridor_point_trans->mutable_xy_ref()->set_x(corridor_point.xy_ref.x());
    corridor_point_trans->mutable_xy_ref()->set_y(corridor_point.xy_ref.y());
    corridor_point_trans->mutable_xy_left()->set_x(corridor_point.xy_left.x());
    corridor_point_trans->mutable_xy_left()->set_y(corridor_point.xy_left.y());
    corridor_point_trans->mutable_xy_right()->set_x(
        corridor_point.xy_right.x());
    corridor_point_trans->mutable_xy_right()->set_y(
        corridor_point.xy_right.y());
    corridor_point_trans->set_theta(corridor_point.theta);
    corridor_point_trans->set_kappa(corridor_point.kappa);
    corridor_point_trans->set_s_ref(corridor_point.s_ref);
    corridor_point_trans->set_l_ref(corridor_point.l_ref);
    corridor_point_trans->set_s(corridor_point.s);
    corridor_point_trans->set_t(corridor_point.t);
  }
}

void ProtoConvertor::RecordCorridorSTBoundaryCounterMap(
    const CorridorInfo& corridor_info,
    PlanningDebugInfo_CorridorInfo* corridor_info_proto) {
  for (const auto& [obs_id, counter] :
       corridor_info.GetSTBoundaryCounterMap()) {
    auto st_boundary_counter_proto =
        corridor_info_proto->add_st_boundary_counter_map();
    st_boundary_counter_proto->set_obs_id(obs_id);
    st_boundary_counter_proto->set_counter(counter);
  }
}

void ProtoConvertor::RecordSTTopologyProto(
    const CorridorInfo& corridor_info,
    PlanningDebugInfo_CorridorInfo_STTopology& st_topology_proto) {
  for (const auto& layer : corridor_info.GetSTTopology().layers) {
    auto layer_proto = st_topology_proto.add_layers();
    for (const auto& obstacle : layer) {
      layer_proto->add_obs_ids(obstacle);
    }
  }
}

void ProtoConvertor::RecordCorridorSTProposalsProto(
    const CorridorInfo& corridor_info,
    PlanningDebugInfo_CorridorInfo* corridor_info_proto) {
  for (const auto& st_proposal : corridor_info.GetSTProposals()) {
    auto st_proposal_proto = corridor_info_proto->add_st_proposals();
    st_proposal_proto->set_cost_total(st_proposal.cost_total);
    for (const auto& st_boundary : st_proposal.st_boundaries) {
      auto st_boundary_proto = st_proposal_proto->add_st_boundaries();
      st_boundary_proto->set_obs_id(st_boundary.obs_id);
      st_boundary_proto->set_is_front(st_boundary.is_front);
    }
    for (const auto& cost : st_proposal.costs) {
      auto cost_proto = st_proposal_proto->add_costs();
      cost_proto->set_reason(cost.reason);
      cost_proto->set_cost(cost.cost);
    }
  }
}

void ProtoConvertor::RecordSTProposalProto(
    const Proposal& proposal,
    PlanningDebugInfo_Proposal_STProposal& st_proposal_proto) {
  for (const auto& st_boundary : proposal.GetSTProposal()) {
    auto st_boundary_debug = st_proposal_proto.add_st_boundaries();
    st_boundary_debug->set_obs_id(std::get<0>(st_boundary)->id());
    st_boundary_debug->set_is_front(std::get<1>(st_boundary));
    st_boundary_debug->set_is_filtered(std::get<2>(st_boundary));
    for (const auto& point : std::get<0>(st_boundary)->points()) {
      st_boundary_debug->add_t(point.x());
      st_boundary_debug->add_s(point.y());
    }
  }
}

void ProtoConvertor::RecordEvaluationProto(
    const Proposal& proposal,
    PlanningDebugInfo_Proposal_Evaluation& evaluation_proto) {
  std::vector<std::string> cost_types;
  for (const auto& costs : proposal.GetCosts()) {
    cost_types.push_back(costs.first);
  }
  std::sort(cost_types.begin(), cost_types.end());
  for (const auto& cost_type : cost_types) {
    const auto& costs = proposal.GetCosts().at(cost_type);
    auto cost_debug = evaluation_proto.add_costs();
    cost_debug->set_cost_type(cost_type);
    for (const auto& cost : costs) {
      cost_debug->add_cost(cost.first);
      cost_debug->add_reason(cost.second);
    }
  }
}

void ProtoConvertor::RecordLonMPCDataProto(
    const Proposal& proposal,
    PlanningDebugInfo_Proposal_LonMPC& lon_mpc_proto) {
  auto lon_mpc_data = proposal.GetLonMPCData();
  RecordMPCDataProto(lon_mpc_data.x, lon_mpc_data.u, lon_mpc_data.u_dot,
                     Eigen::MatrixXd(), lon_mpc_data.x_ref, lon_mpc_data.u_ref,
                     lon_mpc_data.constraints, lon_mpc_data.slacks,
                     lon_mpc_data.dt, lon_mpc_data.n_steps, lon_mpc_data.u_prev,
                     lon_mpc_data.dt_prev, lon_mpc_data.names,
                     *lon_mpc_proto.mutable_mpc_data());
  RecordSpeedLimitSetProto(lon_mpc_data.dt, lon_mpc_data.speed_limit_set,
                           *lon_mpc_proto.mutable_speed_limit_set());
  lon_mpc_proto.set_is_stop_hold(lon_mpc_data.is_stop_hold);
}

void ProtoConvertor::RecordLatMPCDataProto(
    const Proposal& proposal,
    PlanningDebugInfo_Proposal_LatMPC& lat_mpc_proto) {
  auto lat_mpc_data = proposal.GetLatMPCData();
  RecordMPCDataProto(lat_mpc_data.x, lat_mpc_data.u, lat_mpc_data.u_dot,
                     lat_mpc_data.u_2, lat_mpc_data.x_ref, lat_mpc_data.u_ref,
                     lat_mpc_data.constraints, lat_mpc_data.slacks,
                     lat_mpc_data.dt, lat_mpc_data.n_steps, lat_mpc_data.u_prev,
                     lat_mpc_data.dt_prev, lat_mpc_data.names,
                     *lat_mpc_proto.mutable_mpc_data());
  RecordTubesProto(lat_mpc_data.tube, *lat_mpc_proto.mutable_tubes());
}

void ProtoConvertor::RecordMPCDataProto(
    const Eigen::MatrixXd& x, const Eigen::MatrixXd& u,
    const Eigen::MatrixXd& u_dot, const Eigen::MatrixXd& u_2,
    const Eigen::MatrixXd& x_ref, const Eigen::MatrixXd& u_ref,
    const MPCData::Constraints& constraints, const MPCData::Slacks& slacks,
    const double dt, const int n_steps, const Eigen::VectorXd& u_prev,
    const double dt_prev, const MPCData::Names& names,
    PlanningDebugInfo_Proposal_MPCData& mpc_data_proto) {
  const int n_nodes = n_steps + 1;
  const Eigen::RowVectorXd t_x = util::TVector(dt, n_nodes);
  const Eigen::RowVectorXd t_u = util::TVector(dt, n_steps);
  Eigen::RowVectorXd t_u_dot(n_steps);
  t_u_dot << -dt_prev, t_u.head(n_steps - 1);
  mpc_data_proto.set_dt_prev(dt_prev);
  for (int i = 0; i < u_prev.size(); ++i) {
    mpc_data_proto.add_u_prev(u_prev(i));
  }

  auto AssignTrajectoryProto =
      [](const std::string& name, const Eigen::RowVectorXd& t,
         const Eigen::RowVectorXd& soln,
         const Eigen::RowVectorXd& ref,  //
         const Eigen::RowVectorXd& constraint_soft_min,
         const Eigen::RowVectorXd& constraint_soft_max,
         const Eigen::RowVectorXd& constraint_stiff_min,
         const Eigen::RowVectorXd& constraint_stiff_max,
         const Eigen::RowVectorXd& constraint_hard_min,
         const Eigen::RowVectorXd& constraint_hard_max,
         const Eigen::RowVectorXd& slack_soft_min,
         const Eigen::RowVectorXd& slack_soft_max,
         const Eigen::RowVectorXd& slack_stiff_min,
         const Eigen::RowVectorXd& slack_stiff_max,
         PlanningDebugInfo_Proposal_MPCData_Trajectory& traj_msg) {
        const int n = t.size();
        traj_msg.set_name(name);
        for (int k = 0; k < n; ++k) {
          traj_msg.add_t(t(k));
          traj_msg.add_soln(soln(k));
          if (ref.size() == n) {
            traj_msg.add_ref(ref(k));
          }
          if (constraint_soft_min.size() == n) {
            traj_msg.mutable_constraints()->add_soft_min(
                constraint_soft_min(k));
          }
          if (constraint_soft_max.size() == n) {
            traj_msg.mutable_constraints()->add_soft_max(
                constraint_soft_max(k));
          }
          if (constraint_stiff_min.size() == n) {
            traj_msg.mutable_constraints()->add_stiff_min(
                constraint_stiff_min(k));
          }
          if (constraint_stiff_max.size() == n) {
            traj_msg.mutable_constraints()->add_stiff_max(
                constraint_stiff_max(k));
          }
          if (constraint_hard_min.size() == n) {
            traj_msg.mutable_constraints()->add_hard_min(
                constraint_hard_min(k));
          }
          if (constraint_hard_max.size() == n) {
            traj_msg.mutable_constraints()->add_hard_max(
                constraint_hard_max(k));
          }
          if (slack_soft_min.size() > 0 && !std::isnan(slack_soft_min(0))) {
            traj_msg.mutable_slacks()->add_soft_min(slack_soft_min(k));
          }
          if (slack_soft_max.size() > 0 && !std::isnan(slack_soft_max(0))) {
            traj_msg.mutable_slacks()->add_soft_max(slack_soft_max(k));
          }
          if (slack_stiff_min.size() > 0 && !std::isnan(slack_stiff_min(0))) {
            traj_msg.mutable_slacks()->add_stiff_min(slack_stiff_min(k));
          }
          if (slack_stiff_max.size() > 0 && !std::isnan(slack_stiff_max(0))) {
            traj_msg.mutable_slacks()->add_stiff_max(slack_stiff_max(k));
          }
        }
      };

  auto ConvertConstraint =
      [](const MPCData::Constraints::Constraint::ConstraintInfo& info,
         const int i_curr) {
        int i_row = -1;
        for (int i = 0; i <= i_curr; ++i) {
          if (info.idx_enabled[i]) {
            i_row++;
          }
        }
        return info.idx_enabled[i_curr] ? info.constraint.row(i_row)
                                        : Eigen::RowVectorXd();
      };

  auto ConvertSlack = [](const Eigen::MatrixXd& slack, const int i_curr) {
    return !std::isnan(slack(i_curr, 0)) ? slack.row(i_curr)
                                         : Eigen::RowVectorXd();
  };

  for (int i = 0; i < static_cast<int>(x.rows()); ++i) {
    auto trajectory = mpc_data_proto.add_trajectories();
    AssignTrajectoryProto(names.x[i], t_x, x.row(i), x_ref.row(i),  //
                          ConvertConstraint(constraints.soft_min.x, i),
                          ConvertConstraint(constraints.soft_max.x, i),
                          ConvertConstraint(constraints.stiff_min.x, i),
                          ConvertConstraint(constraints.stiff_max.x, i),
                          ConvertConstraint(constraints.hard_min.x, i),
                          ConvertConstraint(constraints.hard_max.x, i),
                          ConvertSlack(slacks.soft_min.x, i),
                          ConvertSlack(slacks.soft_max.x, i),
                          ConvertSlack(slacks.stiff_min.x, i),
                          ConvertSlack(slacks.stiff_max.x, i),  //
                          *trajectory);
  }

  for (int i = 0; i < static_cast<int>(u.rows()); ++i) {
    auto trajectory = mpc_data_proto.add_trajectories();
    AssignTrajectoryProto(names.u[i], t_u, u.row(i), u_ref.row(i),  //
                          ConvertConstraint(constraints.soft_min.u, i),
                          ConvertConstraint(constraints.soft_max.u, i),
                          ConvertConstraint(constraints.stiff_min.u, i),
                          ConvertConstraint(constraints.stiff_max.u, i),
                          ConvertConstraint(constraints.hard_min.u, i),
                          ConvertConstraint(constraints.hard_max.u, i),
                          ConvertSlack(slacks.soft_min.u, i),
                          ConvertSlack(slacks.soft_max.u, i),
                          ConvertSlack(slacks.stiff_min.u, i),
                          ConvertSlack(slacks.stiff_max.u, i),  //
                          *trajectory);
  }

  for (int i = 0; i < static_cast<int>(u_dot.rows()); ++i) {
    auto trajectory = mpc_data_proto.add_trajectories();
    AssignTrajectoryProto(names.u_dot[i], t_u_dot, u_dot.row(i),
                          Eigen::RowVectorXd(),  //
                          ConvertConstraint(constraints.soft_min.u_dot, i),
                          ConvertConstraint(constraints.soft_max.u_dot, i),
                          ConvertConstraint(constraints.stiff_min.u_dot, i),
                          ConvertConstraint(constraints.stiff_max.u_dot, i),
                          ConvertConstraint(constraints.hard_min.u_dot, i),
                          ConvertConstraint(constraints.hard_max.u_dot, i),
                          ConvertSlack(slacks.soft_min.u_dot, i),
                          ConvertSlack(slacks.soft_max.u_dot, i),
                          ConvertSlack(slacks.stiff_min.u_dot, i),
                          ConvertSlack(slacks.stiff_max.u_dot, i),  //
                          *trajectory);
  }

  for (int i = 0; i < static_cast<int>(u_2.rows()); ++i) {
    auto trajectory = mpc_data_proto.add_trajectories();
    AssignTrajectoryProto(
        names.u_2[i], t_u, u_2.row(i), Eigen::RowVectorXd(),  //
        Eigen::RowVectorXd(), Eigen::RowVectorXd(), Eigen::RowVectorXd(),
        Eigen::RowVectorXd(), Eigen::RowVectorXd(), Eigen::RowVectorXd(),
        Eigen::RowVectorXd(), Eigen::RowVectorXd(), Eigen::RowVectorXd(),
        Eigen::RowVectorXd(), *trajectory);
  }
}

void ProtoConvertor::RecordSpeedLimitSetProto(
    const double dt, const std::vector<SpeedLimit>& speed_limit_set,
    google::protobuf::RepeatedPtrField<PlanningDebugInfo_SpeedLimit>&
        speed_limit_set_proto) {
  const int n_size = speed_limit_set.size();
  const Eigen::VectorXd t = util::TVector(dt, n_size);
  for (int k = 0; k < n_size; ++k) {
    auto speed_limit_proto = speed_limit_set_proto.Add();
    speed_limit_proto->set_name(util::ConvertToTimeString(t(k)));
    for (int i = 0; i < static_cast<int>(
                            speed_limit_set.at(k).speed_limit_points().size());
         ++i) {
      speed_limit_proto->add_s(
          speed_limit_set.at(k).speed_limit_points().at(i).first);
      speed_limit_proto->add_v(
          speed_limit_set.at(k).speed_limit_points().at(i).second);
    }
  }
}

void ProtoConvertor::RecordTubesProto(
    const Tube& tube, PlanningDebugInfo_Proposal_LatMPC_Tubes& tubes_proto) {
  for (int k = 0; k < static_cast<int>(tube.pts.size()); k++) {
    tubes_proto.add_t(tube.pts.at(k).t);
    auto xy_left_ref = tubes_proto.mutable_ref()->add_xy_left();
    auto xy_right_ref = tubes_proto.mutable_ref()->add_xy_right();
    auto xy_left_soft = tubes_proto.mutable_soft()->add_xy_left();
    auto xy_left_stiff = tubes_proto.mutable_stiff()->add_xy_left();
    auto xy_left_hard = tubes_proto.mutable_hard()->add_xy_left();
    auto xy_right_soft = tubes_proto.mutable_soft()->add_xy_right();
    auto xy_right_stiff = tubes_proto.mutable_stiff()->add_xy_right();
    auto xy_right_hard = tubes_proto.mutable_hard()->add_xy_right();
    xy_left_ref->set_x(tube.pts.at(k).xy_left_ref.x());
    xy_left_ref->set_y(tube.pts.at(k).xy_left_ref.y());
    xy_right_ref->set_x(tube.pts.at(k).xy_right_ref.x());
    xy_right_ref->set_y(tube.pts.at(k).xy_right_ref.y());
    xy_left_soft->set_x(tube.pts.at(k).xy_left_soft.x());
    xy_left_soft->set_y(tube.pts.at(k).xy_left_soft.y());
    xy_left_stiff->set_x(tube.pts.at(k).xy_left_stiff.x());
    xy_left_stiff->set_y(tube.pts.at(k).xy_left_stiff.y());
    xy_left_hard->set_x(tube.pts.at(k).xy_left_hard.x());
    xy_left_hard->set_y(tube.pts.at(k).xy_left_hard.y());
    xy_right_soft->set_x(tube.pts.at(k).xy_right_soft.x());
    xy_right_soft->set_y(tube.pts.at(k).xy_right_soft.y());
    xy_right_stiff->set_x(tube.pts.at(k).xy_right_stiff.x());
    xy_right_stiff->set_y(tube.pts.at(k).xy_right_stiff.y());
    xy_right_hard->set_x(tube.pts.at(k).xy_right_hard.x());
    xy_right_hard->set_y(tube.pts.at(k).xy_right_hard.y());
  }
}

void ProtoConvertor::RecordTrajectoryProto(
    const Proposal& proposal,
    PlanningDebugInfo_Proposal_Trajectory& trajectory_proto) {
  for (const auto point : proposal.GetTrajectory()) {
    trajectory_proto.add_t(point.relative_time());
    trajectory_proto.add_x(point.path_point().x());
    trajectory_proto.add_y(point.path_point().y());
    trajectory_proto.add_theta(point.path_point().theta());
    trajectory_proto.add_kappa(point.path_point().kappa());
    trajectory_proto.add_s(point.path_point().s());
    trajectory_proto.add_v(point.v());
    trajectory_proto.add_a(point.a());
    trajectory_proto.add_j(point.da());
    trajectory_proto.add_delta(point.steer());
    trajectory_proto.add_delta_dot(point.steer_rate());
  }
}

void ProtoConvertor::RecordHeader(
    const zark::planning::ADCTrajectory& trajectory,
    zark::common::ExternedHeader* header) {
  header->mutable_common_header()->set_timestamp_nano(
      trajectory.header().timestamp_sec() * 1.0e9);
  header->mutable_common_header()->set_frame_id(trajectory.header().frame_id());
  header->set_version(1);
  header->set_module_name("Planning");
  header->mutable_common_header()->set_sequence_num(
      trajectory.header().sequence_num() % KNumRollCounter);
  header->mutable_status()->set_error_code(static_cast<zark::common::ErrorCode>(
      trajectory.header().status().error_code()));
  header->mutable_status()->set_msg(trajectory.header().status().msg());
}

void ProtoConvertor::RecordTrajectoryPublished(
    const zark::planning::ADCTrajectory& trajectory,
    zark::planning::PlanningOutputMsg& planning_output_proto) {
  if (trajectory.trajectory_point().size() > 0) {
    if (fabs(trajectory.trajectory_point().at(0).path_point().x()) < 0.01 &&
        fabs(trajectory.trajectory_point().at(0).path_point().y()) < 0.01 &&
        fabs(trajectory.trajectory_point().at(0).a()) < 0.01) {
      AINFO << " get trajectory point is zero and a is zero";
    }
  }
  for (std::size_t i = 0; i < trajectory.trajectory_point().size(); i++) {
    zark::planning::TrajectoryPointMsg* trajectory_point =
        planning_output_proto.add_trajectory_published();
    zark::planning::Point3d* point = trajectory_point->mutable_point();
    TrajectoryPoint input_trajectory_point =
        trajectory.trajectory_point().at(i);
    point->set_x(input_trajectory_point.path_point().x());
    point->set_y(input_trajectory_point.path_point().y());
    point->set_z(input_trajectory_point.path_point().z());
    trajectory_point->set_s(input_trajectory_point.path_point().s());
    trajectory_point->set_v(input_trajectory_point.v());
    trajectory_point->set_a(input_trajectory_point.a());
    trajectory_point->set_theta(input_trajectory_point.path_point().theta());
    trajectory_point->set_kappa(input_trajectory_point.path_point().kappa());
    trajectory_point->set_dkappa(input_trajectory_point.path_point().dkappa());
    trajectory_point->set_t(input_trajectory_point.relative_time());
  }
}

void ProtoConvertor::RecordTrajectoryPolyfit(
    const zark::planning::ADCTrajectory& trajectory,
    zark::planning::PlanningOutputMsg& planning_output_proto) {
  auto lon_coeff = planning_output_proto.mutable_trajectory_polyfit()
                       ->mutable_lon_fit_coeff();
  lon_coeff->Clear();
  auto lat_coeff = planning_output_proto.mutable_trajectory_polyfit()
                       ->mutable_lat_fit_coeff();
  lat_coeff->Clear();

  if (trajectory.trajectory_point().empty()) {  // return when traj is empty
    planning_output_proto.mutable_trajectory_polyfit()->set_traj_length(0.0);
    planning_output_proto.mutable_trajectory_polyfit()->set_is_valid(false);
    return;
  }

  auto traj_polyfit = trajectory.traj_polyfit_out();
  for (int i = 0; i < traj_polyfit.lat_coefficients.size(); i++) {
    lat_coeff->Add(traj_polyfit.lat_coefficients(i));
  }

  for (int i = 0; i < traj_polyfit.lon_coefficients.size(); i++) {
    lon_coeff->Add(traj_polyfit.lon_coefficients(i));
  }

  // keep lat and lon size == 6
  while (lon_coeff->size() < 6) {
    lon_coeff->Add(0.0);
  }
  while (lat_coeff->size() < 6) {
    lat_coeff->Add(0.0);
  }

  planning_output_proto.mutable_trajectory_polyfit()->set_traj_length(
      traj_polyfit.length);
  planning_output_proto.mutable_trajectory_polyfit()->set_is_valid(
      traj_polyfit.is_valid);
}

void ProtoConvertor::RecordPlanningStatus(
    const zark::planning::ADCTrajectory& trajectory,
    zark::planning::PlanningOutputMsg& planning_output_proto) {
  planning_output_proto.set_planning_status(trajectory.planning_status());
}

void ProtoConvertor::RecordLocalRoutePoint(
    const LocalRoutePoint& point, const zark::planning::hdmap::Path& map_path,
    const int32_t path_point_index,
    zark::reference_line_proto::ReferenceLineInfo* const out_line) {
  zark::reference_line_proto::PathPoint* const out_point =
      out_line->add_path_points();
  auto point_xy = out_point->mutable_base_point();
  out_point->set_theta(point.heading());
  out_point->set_kappa(point.kappa());
  out_point->set_dkappa(point.dkappa());
  point_xy->set_x(point.x());
  point_xy->set_y(point.y());
  double point_s = map_path.GetSFromIndex(
      zark::planning::hdmap::InterpolatedIndex(path_point_index, 0.0));
  out_point->set_s(point_s);
  double left_lane_width, right_lane_width;
  map_path.GetLaneWidth(point_s, &left_lane_width, &right_lane_width);
  out_point->set_left_width(left_lane_width);
  out_point->set_right_width(right_lane_width);
}

void ProtoConvertor::RecordLaneSegment(
    const zark::planning::hdmap::LaneSegment& lane_seg,
    zark::reference_line_proto::ReferenceLineInfo* const out_line) {
  if (lane_seg.lane) {
    zark::reference_line_proto::LaneInfo* const lane_info =
        out_line->add_lane_info();
    std::string map_lane_id = lane_seg.lane->lane().id().id();
    lane_info->set_s_begin(lane_seg.start_s);
    lane_info->set_s_end(lane_seg.end_s);
    lane_info->set_lane_id(map_lane_id);
  }
}

}  // namespace common
}  // namespace planning
}  // namespace zark
