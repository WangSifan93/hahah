/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file: longitudinal_decider.cc
 **/

#include "apps/planning/src/tasks/deciders/longitudinal_decider/longitudinal_decider.h"

#include <algorithm>
#include <memory>

#include "apps/planning/src/common/proposal.h"
#include "apps/planning/src/decision/longitudinal/data_type.h"

namespace zark {
namespace planning {

using ::common::Status;
static constexpr double kKmPHToMPS = 3.6;

LongitudinalDecider::LongitudinalDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Task(config, injector) {
  const auto& long_decider_config =
      config_.task_config().longitudinal_decider_config;
  speed_limit_builder_ = std::make_unique<SpeedLimitBuilder>(
      long_decider_config.speed_limit_config);
  obs_sl_boundary_builder_ = std::make_unique<ObsSLBoundaryBuilder>();
  st_graph_builder_ = std::make_unique<STGraphBuilder>(
      config_.task_config().longitudinal_decider_config.st_graph_config);
  st_topology_builder_ = std::make_unique<STTopologyBuilder>(
      STTopologyBuilder(long_decider_config.st_topology_config));
  st_proposal_builder_ = std::make_unique<STProposalBuilder>(
      STProposalBuilder(long_decider_config.st_proposal_config));
  InitSTBoundaryCounterMaps();
}

Status LongitudinalDecider::Execute(Frame* frame) {
  frame_ = frame;

  const double v_ego = frame->PlanningStartPoint().v();
  const double goal_cruise_speed = frame_->local_view()
                                       .fct_output->fct_out_bus_accinfosts()
                                       .fct_out_v_accactsetspd_sg() /
                                   kKmPHToMPS;
  UpdateSTBoundaryCounterMaps(frame_->GetCorridorInfos());
  for (CorridorInfo& current_corridor_info :
       *(frame_->MutableCorridorInfos())) {
    const std::unordered_map<std::string, SpeedLimit> speed_limit_map =
        speed_limit_builder_->BuildSpeedLimitMap(
            current_corridor_info.GetLocalRoute(),
            current_corridor_info.GetCorridor(), v_ego, goal_cruise_speed);
    current_corridor_info.SetSpeedLimitMap(speed_limit_map);

    const std::unordered_map<const Obstacle*, SLTrajectory>
        obs_sl_boundary_map = obs_sl_boundary_builder_->BuildObsSLBoundaryMap(
            current_corridor_info.GetCorridor(),
            current_corridor_info.GetObstacleMap().Items());
    current_corridor_info.SetObsSLBoundayMap(obs_sl_boundary_map);

    std::vector<std::pair<const Obstacle*, bool>> lateral_obstacles;
    std::vector<STBoundary> st_graph;
    st_graph_builder_->BuildSTGraph(
        current_corridor_info.GetCorridor(),
        current_corridor_info.GetObstacleMap(), obs_sl_boundary_map, st_graph,
        lateral_obstacles,
        st_boundary_counter_maps_[current_corridor_info.GetType()]);
    current_corridor_info.SetSTGraph(st_graph);
    current_corridor_info.SetLateralObstacles(lateral_obstacles);
    current_corridor_info.SetSTBoundaryCounterMap(
        st_boundary_counter_maps_[current_corridor_info.GetType()]);

    const STTopology st_topology = st_topology_builder_->BuildSTTopology(
        current_corridor_info.GetSTGraph(), v_ego);
    current_corridor_info.SetSTTopology(RecordSTTopology(st_topology));

    std::vector<CorridorInfo::STProposal> st_proposals_record;
    const std::vector<STProposal> st_proposals =
        st_proposal_builder_->BuildSTProposals(
            st_topology, v_ego, current_corridor_info.GetType(),
            frame_->IsNearIntersection(), st_proposals_record);
    current_corridor_info.SetSTProposals(st_proposals_record);

    for (auto st_proposal : st_proposals) {
      Proposal proposal;
      proposal.SetCorridorInfo(&current_corridor_info);
      proposal.SetSTProposal(st_proposal);
      frame_->MutableProposals()->emplace_back(proposal);
    }
  }
  return Status::OK();
}

CorridorInfo::STTopology LongitudinalDecider::RecordSTTopology(
    const STTopology& st_topology) {
  CorridorInfo::STTopology st_topology_record;
  for (const auto& layer_curr : st_topology) {
    std::vector<std::string> layer;
    for (const auto& st_boundary : layer_curr) {
      layer.emplace_back(st_boundary->id());
    }
    st_topology_record.layers.emplace_back(layer);
  }
  return st_topology_record;
}

void LongitudinalDecider::InitSTBoundaryCounterMaps() {
  for (auto type : CorridorInfo::corridor_types) {
    st_boundary_counter_maps_[type] = std::unordered_map<std::string, int>();
  }
}

void LongitudinalDecider::UpdateSTBoundaryCounterMaps(
    const std::vector<CorridorInfo>& corridor_infos) {
  std::unordered_set<CorridorInfo::Type> corridor_type_set;
  for (const auto& corridor_info : corridor_infos) {
    corridor_type_set.emplace(corridor_info.GetType());
  }
  for (const auto& type : CorridorInfo::corridor_types) {
    if (!corridor_type_set.count(type)) {
      st_boundary_counter_maps_[type].clear();
    }
  }
}

}  // namespace planning
}  // namespace zark
