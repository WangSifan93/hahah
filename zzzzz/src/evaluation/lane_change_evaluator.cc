/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file lane_change_evaluation.cc
 **/

#include "apps/planning/src/evaluation/lane_change_evaluator.h"
#include "apps/planning/src/common/log.h"
#include "apps/planning/src/evaluation/const_values.h"

namespace zark {
namespace planning {

LaneChangeEvaluator::LaneChangeEvaluator(
    EvaluationDeciderConfig::LaneChange config)
    : config_(config) {}

Costs LaneChangeEvaluator::Evaluate(const Proposal& proposal,
                                    const CorridorInfo::Type last_corridor_type,
                                    const double t_staging) {
  if (proposal.GetCorridorInfo() == nullptr) {
    ZSLOG_ERROR << "CorridorInfo should not be empty in Proposal !!!";
    return Costs();
  }

  Costs costs;
  const auto c_type = proposal.GetCorridorInfo()->GetType();
  const auto& mission = proposal.GetCorridorInfo()->GetMission();
  EvaluateLaneChangeUrgencyCost(mission, proposal.GetLonMPCData(), c_type,
                                t_staging, costs);
  EvaluateLaneChangeAbortCost(c_type, last_corridor_type, costs);
  EvaluateLaneChangeStateCost(c_type, mission, costs);

  return costs;
}

void LaneChangeEvaluator::EvaluateLaneChangeUrgencyCost(
    const Mission& mission, const LonMPCData& lon_mpc_data,
    const CorridorInfo::Type proposal_corridor_type, const double t_staging,
    Costs& costs) {
  const double urgency_cost = config_.urgency_cost_weight * mission.lc_urgency;
  const std::string reason = "urgency cost ";
  const double staging_multiplier =
      t_staging / (lon_mpc_data.dt * lon_mpc_data.n_steps);
  switch (proposal_corridor_type) {
    case CorridorInfo::Type::LANE_KEEP:
      if (urgency_cost > kCostEpsilon) {
        costs.emplace_back(std::make_pair(urgency_cost, reason));
      }
      break;
    case CorridorInfo::Type::STAGING:
      if (staging_multiplier * urgency_cost > kCostEpsilon) {
        costs.emplace_back(std::make_pair(
            staging_multiplier * urgency_cost + config_.staging_cost_addition,
            reason));
      }
      break;
    default:
      break;
  }
}

void LaneChangeEvaluator::EvaluateLaneChangeAbortCost(
    const CorridorInfo::Type proposal_corridor_type,
    const CorridorInfo::Type last_corridor_type, Costs& costs) {
  if ((last_corridor_type == CorridorInfo::Type::LANE_CHANGE) &&
      (proposal_corridor_type != CorridorInfo::Type::LANE_CHANGE)) {
    if (config_.abortion_cost > kCostEpsilon) {
      const std::string reason = "lane change abort cost ";
      costs.emplace_back(std::make_pair(config_.abortion_cost, reason));
    }
  }
}

void LaneChangeEvaluator::EvaluateLaneChangeStateCost(
    const CorridorInfo::Type corridor_type, const Mission& mission,
    Costs& costs) {
  const bool is_lc = corridor_type == CorridorInfo::Type::LANE_CHANGE;
  const bool is_staging = corridor_type == CorridorInfo::Type::STAGING;
  const bool cur_tgt_lane = mission.target_reference_line ==
                            Mission::TargetReflineType::REF_CURRENT_LANE;

  // if lc proposal & target lane is current lane
  if ((is_lc || is_staging) && cur_tgt_lane) {
    costs.emplace_back(config_.last_lc_finished_cost, "last lc finished cost");
  }

  // check 'is_lc_ready' signal and current proposal type
  if (!mission.is_lc_ready && is_lc) {
    costs.emplace_back(config_.lc_forbidden_cost, "lc forbidden cost");
  }
}

}  // namespace planning
}  // namespace zark
