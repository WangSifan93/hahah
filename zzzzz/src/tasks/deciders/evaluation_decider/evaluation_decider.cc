/******************************************************************************
 * Copyright 2024 The zpilot . All Rights Reserved.
 *****************************************************************************/

/**
 * @file evaluation_decider.cc
 **/
#include "apps/planning/src/tasks/deciders/evaluation_decider/evaluation_decider.h"

namespace zark {
namespace planning {

using ::common::Status;
using ::math::Polygon2d;
using ::math::Vec2d;

EvaluationDecider::EvaluationDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector),
      safety_evaluator_(SafetyEvaluator(
          config_.task_config().evaluation_decider_config.safety)),
      comfort_evaluator_(ComfortEvaluator(
          config_.task_config().evaluation_decider_config.comfort)),
      progress_evaluator_(ProgressEvaluator(
          config_.task_config().evaluation_decider_config.progress)),
      similarity_evaluator_(SimilarityEvaluator(
          config_.task_config().evaluation_decider_config.similarity)),
      lane_change_evaluator_(
          config_.task_config().evaluation_decider_config.lane_change) {}

Status EvaluationDecider::Execute(Frame* frame) {
  CHECK_NOTNULL(frame);
  auto AddCostTable = [](const std::string category, const Costs& costs,
                         std::unordered_map<std::string, Costs>* cost_table) {
    if (!costs.empty()) {
      cost_table->emplace(category, costs);
    }
  };
  auto SumCosts = [](const Costs& costs, const double max_cost) {
    double cost_sum = 0.0;
    if (!costs.empty()) {
      for (const auto& cost : costs) {
        cost_sum += cost.first;
      }
    }
    return cost_sum > max_cost ? max_cost : cost_sum;
  };
  auto& proposals = *frame->MutableProposals();
  if (!progress_evaluator_.CalculateMaxDistance(proposals)) {
    const std::string msg = "max proposal distance too small (~0.0) !";
    AWARN << msg;
  }
  DiscretizedTrajectory traj_prev;
  CorridorInfo::Type corridor_type_prev = CorridorInfo::Type::LANE_KEEP;
  auto* frame_prev = injector_->frame_history()->Latest();
  if (frame_prev && !frame_prev->GetProposals().empty()) {
    traj_prev = frame_prev->GetProposals().front().GetTrajectory();
    corridor_type_prev =
        frame_prev->GetProposals().front().GetCorridorInfo()->GetType();
  }
  for (auto& proposal : proposals) {
    auto cost_table = proposal.GetMutableCosts();
    const Costs safety_cost = safety_evaluator_.Evaluate(
        proposal, proposal.GetCorridorInfo()->GetObstacleMap().Items());
    const Costs comfort_cost = comfort_evaluator_.Evaluate(proposal);
    const Costs progress_cost = progress_evaluator_.Evaluate(proposal);
    const Costs similarity_cost =
        similarity_evaluator_.Evaluate(proposal, traj_prev);
    const Costs lane_change_cost = lane_change_evaluator_.Evaluate(
        proposal, corridor_type_prev,
        config_.task_config().lateral_decider_config.corridor.t_staging);
    const auto& cost_limit =
        config_.task_config().evaluation_decider_config.cost_limit;
    const double cost_sum =
        SumCosts(safety_cost, cost_limit.safety_max_cost) +
        SumCosts(comfort_cost, cost_limit.comfort_max_cost) +
        SumCosts(progress_cost, cost_limit.progress_max_cost) +
        SumCosts(similarity_cost, cost_limit.default_max_cost) +
        SumCosts(lane_change_cost, cost_limit.default_max_cost);

    AddCostTable("safety", safety_cost, cost_table);
    AddCostTable("comfort", comfort_cost, cost_table);
    AddCostTable("progress", progress_cost, cost_table);
    AddCostTable("similarity", similarity_cost, cost_table);
    AddCostTable("lane_change", lane_change_cost, cost_table);
    AddCostTable("Total", {std::make_pair(cost_sum, "")}, cost_table);
  }
  std::sort(proposals.begin(), proposals.end(),
            [](const Proposal& p_1, const Proposal& p_2) {
              return p_1.GetCosts().at("Total").at(0).first <
                     p_2.GetCosts().at("Total").at(0).first;
            });

  frame->SetLCGap(
      ComputeLaneChangeGap(frame->GetMission(), *frame->MutableProposals()));

  return Status::OK();
}

GapChoiceInfo EvaluationDecider::ComputeLaneChangeGap(
    const Mission& mission, std::vector<Proposal>& proposals) {
  const bool is_lc_ready = mission.is_lc_ready;
  const bool is_lc_requested =
      (mission.lc_request == Mission::LaneChangeRequest::LC_LEFT ||
       mission.lc_request == Mission::LaneChangeRequest::LC_RIGHT);
  bool is_lc_gap_exist = false;
  uint32_t lc_block_front_obj_id = 0;
  uint32_t lc_block_back_obj_id = 0;
  Proposal lc_proposal;

  if (is_lc_requested) {
    for (const auto& proposal : proposals) {
      if (proposal.GetCorridorInfo()->GetType() ==
          CorridorInfo::Type::LANE_CHANGE) {
        is_lc_gap_exist = true;
        lc_proposal = proposal;
        break;
      }
    }
  }

  // find the objects influence lc_proposal being selected
  if (is_lc_ready && is_lc_gap_exist) {
    if (proposals.front().GetCorridorInfo()->GetType() !=
        CorridorInfo::Type::LANE_CHANGE) {
      double min_dist = std::numeric_limits<double>::max();
      double max_dist = std::numeric_limits<double>::lowest();
      for (const auto& [st_boundary, is_front, is_filtered] :
           lc_proposal.GetSTProposal()) {
        if (is_front) {
          if (st_boundary->min_s() < min_dist) {
            min_dist = st_boundary->min_s();
            lc_block_front_obj_id = std::stoi(st_boundary->id());
          }
        } else {
          if (st_boundary->min_s() > max_dist) {
            max_dist = st_boundary->min_s();
            lc_block_back_obj_id = std::stoi(st_boundary->id());
          }
        }
      }
    }
  }

  return GapChoiceInfo{is_lc_gap_exist, lc_block_front_obj_id,
                       lc_block_back_obj_id};
}

}  // namespace planning
}  // namespace zark
