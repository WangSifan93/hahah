/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file progress_evaluator.cc
 **/

#include "apps/planning/src/evaluation/progress_evaluator.h"
#include "apps/planning/src/evaluation/const_values.h"

namespace zark {
namespace planning {

constexpr int kIdxS = 0;

ProgressEvaluator::ProgressEvaluator(EvaluationDeciderConfig::Progress config)
    : config_(config),
      progress_cost_table_(LookupTable(config.progress_cost_table)) {}

Costs ProgressEvaluator::Evaluate(const Proposal& proposal) {
  Costs costs;
  const std::string max_distance_reason =
      "max proposal distance is " + std::to_string(max_distance_);
  const double progress_distance =
      proposal.GetLonMPCData().x.row(kIdxS).tail(1)(0);
  const std::string progress_distance_reason =
      "the progress distance is " + std::to_string(progress_distance);
  const double distance_diff = max_distance_ - progress_distance;
  const double distance_diff_ratio =
      max_distance_ > kCostEpsilon ? distance_diff / max_distance_ : 0.0;
  double diverge_distance = kMaxForwardDistacne;
  bool has_lc_request = false;
  bool is_routing_lc = false;
  bool is_lc_proposal = false;
  if (proposal.GetCorridorInfo() != nullptr) {
    diverge_distance = proposal.GetCorridorInfo()
                           ->GetLocalRoute()
                           .GetRouteLCInfo()
                           .dist_to_next_lc_pt.at(0);
    has_lc_request = proposal.GetCorridorInfo()->GetMission().lc_request !=
                     Mission::LaneChangeRequest::LANE_KEEP;
    is_routing_lc = proposal.GetCorridorInfo()->GetMission().lc_type ==
                    Mission::LaneChangeType::ROUTING_LC;
    is_lc_proposal = proposal.GetCorridorInfo()->GetType() ==
                     CorridorInfo::Type::LANE_CHANGE;
  }
  if (has_lc_request && is_routing_lc && !is_lc_proposal &&
      diverge_distance < progress_distance) {
    if (config_.exceed_diverged_pt_cost > kCostEpsilon) {
      const std::string reason = max_distance_reason + "m. " +
                                 progress_distance_reason + "m. " +
                                 "proposal exceed diverge point !!!";
      costs.emplace_back(
          std::make_pair(config_.exceed_diverged_pt_cost, reason));
    }
    return costs;
  }
  const double progress_cost =
      progress_cost_table_.Evaluate(distance_diff_ratio);
  if (progress_cost > kCostEpsilon) {
    const std::string reason =
        max_distance_reason + "m. " + progress_distance_reason + "m. ";
    costs.emplace_back(std::make_pair(progress_cost, reason));
    return costs;
  }
  return costs;
}

bool ProgressEvaluator::CalculateMaxDistance(
    const std::vector<Proposal>& proposals) {
  max_distance_ = 0.0;
  for (const auto& proposal : proposals) {
    const double progress_distance =
        proposal.GetLonMPCData().x.row(kIdxS).tail(1)(0);
    max_distance_ = std::max(max_distance_, progress_distance);
  }
  if (max_distance_ > kCostEpsilon) {
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace zark
