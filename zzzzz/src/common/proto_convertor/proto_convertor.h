/******************************************************************************
 * Copyright 2017 The zpilot Authors. All Rights Reserved.
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
 * @proto_convertor.h
 **/

#pragma once

#include <Eigen/Core>
#include <vector>

#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/mission.h"
#include "apps/planning/src/common/mpc_data.h"
#include "apps/planning/src/common/proposal.h"
#include "apps/planning/src/common/speed_limit.h"
#include "apps/planning/src/planning_msgs/planning.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {
namespace common {

class ProtoConvertor {
 public:
  ProtoConvertor() = default;

  virtual ~ProtoConvertor() = default;

  /**
   * @brief Record planning debug info
   *
   * @param corridor_infos a set of CorridorInfos
   * @param proposals a set of proposals
   * @param corridor_info_prev a set of previous CorridorInfo
   * @param mission a set of missions
   * @param latency_stats a set of latency status
   * @param is_replan the signal of whether replan
   * @param replan_reason the reason for replan
   * @param debug_info proto of proposals msg
   */
  static void RecordPlanningDebugInfo(
      const std::vector<CorridorInfo>& corridor_infos,
      const std::vector<Proposal>& proposals,
      const CorridorInfo* corridor_info_prev, const Mission& mission,
      const LatencyStats& latency_stats, const bool is_replan,
      const std::string& replan_reason,
      PlanningDebugInfo& planning_debug_info_proto);

  /**
   * @brief Record planning output
   *
   * @param trajectory trajectory from planner
   * @param planning_output_proto proto of planning output
   */
  static void RecordPlanningOutput(
      const zark::planning::ADCTrajectory& trajectory,
      zark::planning::PlanningOutputMsg& planning_output_proto);

  /**
   * @brief Record local route
   *
   * @param local_routes list of local_routes
   * @param ref_lines proto of reference lines
   */
  static void RecordLocalRoute(
      const std::list<zark::planning::LocalRoute>& local_routes,
      zark::reference_line_proto::ReferenceLines& ref_lines);

 private:
  /**
   * @brief Record latency stats into corresponding proto message.
   *
   * @param latency_stats
   * @param planning_debug_info_proto
   */
  static void RecordLatencyStatsProto(
      const LatencyStats& latency_stats,
      PlanningDebugInfo& planning_debug_info_proto);
  /**
   * @brief Record the mission data struct into the corresponding proto message.
   *
   * @param mission
   * @param planning_debug_info_proto
   */
  static void RecordMissionProto(const Mission& mission,
                                 PlanningDebugInfo& planning_debug_info_proto);

  /**
   * @brief Record the corridor infos data struct into the corresponding proto
   * message.
   *
   * @param corridor_infos a set of CorridorInfos
   * @param corridor_info_proto proto of corrifor_info
   */
  static void RecordCorridorInfosProto(
      const std::vector<CorridorInfo>& corridor_infos,
      PlanningDebugInfo& planning_debug_info_proto);

  /**
   * @brief Record the proposals data struct into the corresponding proto
   * message.
   *
   * @param proposals a set of proposals
   * @param planning_debug_info_proto proto of planning debug info
   */
  static void RecordProposalsProto(
      const std::vector<Proposal>& proposals,
      PlanningDebugInfo& planning_debug_info_proto);
  /**
   * @brief Record the corridor info data struct into the corresponding proto
   * message.
   *
   * @param corridor_info class of CorridorInfo
   * @param corridor_info_proto proto of corridor info data
   */
  static void RecordCorridorInfoProto(
      const CorridorInfo& corridor_info,
      PlanningDebugInfo_CorridorInfo* corridor_info_proto);

  /**
   * @brief Record the proposal data struct into the corresponding proto
   * message.
   *
   * @param proposal class of Proposal
   * @param proposal_proto proto of proposal data
   */
  static void RecordProposalProto(const Proposal& proposal,
                                  PlanningDebugInfo_Proposal* proposal_proto);

  /**
   * @brief Convert corridor type to string.
   *
   * @param type corridor type
   * @param std::string corridor type in string
   */
  static std::string ConvertCorridorTypeToString(const CorridorInfo::Type type);

  /**
   * @brief Record the speed limit map data struct into the corresponding proto
   * message.
   *
   * @param corridor_info class of CorridorInfo
   * @param corridor_info_proto proto of corridor info data
   */
  static void RecordSpeedLimitMapProto(
      const CorridorInfo& corridor_info,
      PlanningDebugInfo_CorridorInfo* corridor_info_proto);

  /**
   * @brief Record the corridor points data struct into the corresponding proto
   * message.
   *
   * @param corridor_info class of CorridorInfo
   * @param corridor_info_proto proto of corridor info data
   */
  static void RecordCorridorPointsProto(
      const CorridorInfo& corridor_info,
      PlanningDebugInfo_CorridorInfo* corridor_info_proto);

  /**
   * @brief Record the corridor info ST boundary counter map data struct into
   * the corresponding proto message.
   *
   * @param corridor_info class of CorridorInfo
   * @param corridor_info_proto proto of corridor info data
   */
  static void RecordCorridorSTBoundaryCounterMap(
      const CorridorInfo& corridor_info,
      PlanningDebugInfo_CorridorInfo* corridor_info_proto);

  /**
   * @brief Record the ST topology data struct into the corresponding proto
   * message.
   *
   * @param corridor_info class of CorridorInfo
   * @param st_topology_proto proto of ST topology data
   */
  static void RecordSTTopologyProto(
      const CorridorInfo& corridor_info,
      PlanningDebugInfo_CorridorInfo_STTopology& st_topology_proto);

  /**
   * @brief Record the corridor info ST proposals data struct into the
   * corresponding proto message.
   *
   * @param corridor_info class of CorridorInfo
   * @param corridor_info_proto proto of corridor info data
   */
  static void RecordCorridorSTProposalsProto(
      const CorridorInfo& corridor_info,
      PlanningDebugInfo_CorridorInfo* corridor_info_proto);

  /**
   * @brief Record the ST proposal data struct into the corresponding proto
   * message.
   *
   * @param proposal class of Proposal
   * @param st_proposal_proto proto of ST proposal data
   */
  static void RecordSTProposalProto(
      const Proposal& proposal,
      PlanningDebugInfo_Proposal_STProposal& st_proposal_proto);

  /**
   * @brief Record the evaluation data struct into the corresponding proto
   * message.
   *
   * @param proposal class of Proposal
   * @param evaluation_proto proto of evaluation data
   */
  static void RecordEvaluationProto(
      const Proposal& proposal,
      PlanningDebugInfo_Proposal_Evaluation& evaluation_proto);

  /**
   * @brief Record the lon MPC data struct into the corresponding proto message.
   *
   * @param proposal class of Proposal
   * @param lon_mpc_proto proto of lon MPC data
   *
   */
  static void RecordLonMPCDataProto(
      const Proposal& proposal,
      PlanningDebugInfo_Proposal_LonMPC& lon_mpc_proto);

  /**
   * @brief Record the lat MPC data struct into the corresponding proto message.
   *
   * @param proposal class of Proposal
   * @param lat_mpc_proto proto of lat MPC data
   *
   */
  static void RecordLatMPCDataProto(
      const Proposal& proposal,
      PlanningDebugInfo_Proposal_LatMPC& lat_mpc_proto);

  /**
   * @brief Record the MPC data struct into the corresponding proto message.
   *
   * @param x The state vector.
   * @param u The control input vector.
   * @param u_dot The time rate of control input vector.
   * @param u_2 The uncontrollable control input vector.
   * @param x_ref The reference state trajectory.
   * @param u_ref The reference control input trajectory.
   * @param constraints Constraints on the MPC data.
   * @param slacks Slack variables for constraints.
   * @param dt The sampling time between time steps [s].
   * @param n_steps The number of time steps in the MPC.
   * @param u_prev The control input applied in the previous planning cycle.
   * @param dt_prev The time elapsed since the previous planning cycle [s].
   * @param names The names of x, u, u_dot for visualization.
   * @param mpc_data_proto Protobuf message to store the mpc data.
   */
  static void RecordMPCDataProto(
      const Eigen::MatrixXd& x, const Eigen::MatrixXd& u,
      const Eigen::MatrixXd& u_dot, const Eigen::MatrixXd& u_2,
      const Eigen::MatrixXd& x_ref, const Eigen::MatrixXd& u_ref,
      const MPCData::Constraints& constraints, const MPCData::Slacks& slacks,
      const double dt, const int n_steps, const Eigen::VectorXd& u_prev,
      const double dt_prev, const MPCData::Names& names,
      PlanningDebugInfo_Proposal_MPCData& mpc_data_proto);

  /**
   * @brief Record the speed_limit_set data into the corresponding proto message
   *
   * @param dt The sampling time between time steps [s].
   * @param speed_limit_set the set of speed limit curves for all time steps
   * @param speed_limit_set_proto Protobuf message to store the speed_limit_set.
   */
  static void RecordSpeedLimitSetProto(
      const double dt, const std::vector<SpeedLimit>& speed_limit_set,
      google::protobuf::RepeatedPtrField<PlanningDebugInfo_SpeedLimit>&
          speed_limit_set_proto);

  /**
   * @brief Record the tube data into the corresponding proto message
   *
   * @param tube the data of tube
   * @param tubes_proto Protobuf message to store the tubes.
   */
  static void RecordTubesProto(
      const Tube& tube, PlanningDebugInfo_Proposal_LatMPC_Tubes& tubes_proto);

  /**
   * @brief Record the trajectory data struct into the corresponding proto
   * message.
   *
   * @param proposal class of Proposal
   * @param trajectory_proto proto of trajectory data
   */
  static void RecordTrajectoryProto(
      const Proposal& proposal,
      PlanningDebugInfo_Proposal_Trajectory& trajectory_proto);

  /**
   * @brief Record header
   *
   * @param trajectory trajectory from planner
   * @param header header of planning output proto
   */
  static void RecordHeader(const zark::planning::ADCTrajectory& trajectory,
                           zark::common::ExternedHeader* header);

  /**
   * @brief Record trajectory published
   *
   * @param trajectory trajectory from planner
   * @param planning_output_proto proto of planning output
   */
  static void RecordTrajectoryPublished(
      const zark::planning::ADCTrajectory& trajectory,
      zark::planning::PlanningOutputMsg& planning_output_proto);

  /**
   * @brief Record trajectory polyfit
   *
   * @param trajectory trajectory from planner
   * @param planning_output_proto proto of planning output
   */
  static void RecordTrajectoryPolyfit(
      const zark::planning::ADCTrajectory& trajectory,
      zark::planning::PlanningOutputMsg& planning_output_proto);

  /**
   * @brief Record planning running status
   *
   * @param trajectory trajectory from planner
   * @param planning_output_proto proto of planning output
   */
  static void RecordPlanningStatus(
      const zark::planning::ADCTrajectory& trajectory,
      zark::planning::PlanningOutputMsg& planning_output_proto);

  static void RecordLocalRoutePoint(
      const LocalRoutePoint& point, const zark::planning::hdmap::Path& map_path,
      const int32_t path_point_index,
      zark::reference_line_proto::ReferenceLineInfo* const out_line);

  static void RecordLaneSegment(
      const zark::planning::hdmap::LaneSegment& lane_seg,
      zark::reference_line_proto::ReferenceLineInfo* const out_line);

 private:
  FRIEND_TEST(ProtoConvertorTest, TestRecordPlanningDebugInfo);
  FRIEND_TEST(ProtoConvertorTest, TestRecordPlanningOutput);
  FRIEND_TEST(ProtoConvertorTest, TestRecordRecordLocalRoute);
  FRIEND_TEST(ProtoConvertorTest, TestRecordLatencyStatsProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordMissionProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordRecordEnvInfosProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordCorridorInfosProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordProposalsProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordCorridorInfoProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordProposalProto);
  FRIEND_TEST(ProtoConvertorTest, TestConvertCorridorTypeToString);
  FRIEND_TEST(ProtoConvertorTest, TestRecordSpeedLimitMapProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordCorridorPointsProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordCorridorSTBoundaryCounterMap);
  FRIEND_TEST(ProtoConvertorTest, TestRecordSTTopologyProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordCorridorSTProposalsProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordSTProposalProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordEvaluationProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordLonMPCDataProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordLatMPCDataProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordMPCDataProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordSpeedLimitSetProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordTubesProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordTrajectoryProto);
  FRIEND_TEST(ProtoConvertorTest, TestRecordHeader);
  FRIEND_TEST(ProtoConvertorTest, TestRecordTrajectoryPublished);
  FRIEND_TEST(ProtoConvertorTest, TestRecordPlanningStatus);
  FRIEND_TEST(ProtoConvertorTest, TestRecordTrajectoryPolyfit);
  FRIEND_TEST(ProtoConvertorTest, TestRecordLocalRoutePoint);
  FRIEND_TEST(ProtoConvertorTest, TestRecordLaneSegment);
};

}  // namespace common
}  // namespace planning
}  // namespace zark
