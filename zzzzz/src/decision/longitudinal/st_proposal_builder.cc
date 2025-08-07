/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file: st_proposal_builder.cc
 **/

#include "apps/planning/src/decision/longitudinal/st_proposal_builder.h"

namespace zark {
namespace planning {

STProposalBuilder::STProposalBuilder(
    const LongitudinalDeciderConfig::STProposalConfig& config) {
  config_ = config;
  narrow_gap_cost_table_ = LookupTable(config_.narrow_gap_cost_table);
  a_acc_max_table_ = LookupTable(config_.a_acc_max_table);
}

std::vector<STProposal> STProposalBuilder::BuildSTProposals(
    const STTopology& st_topology, const double v_extrap,
    const CorridorInfo::Type& type, const bool is_near_intersection,
    std::vector<CorridorInfo::STProposal>& st_proposals_record) {
  std::vector<STProposal> st_proposals =
      GenerateSTProposals(st_topology, type, is_near_intersection);

  std::vector<std::pair<STProposal, double>> cost_proposals;
  std::unordered_map<const STBoundary*, double> cost_map_front;
  std::unordered_map<const STBoundary*, double> cost_map_rear;
  SortAndFilterSTProposals(st_topology, v_extrap, cost_map_front, cost_map_rear,
                           cost_proposals, st_proposals);

  st_proposals_record =
      RecordSTProposals(cost_map_front, cost_map_rear, cost_proposals);
  return st_proposals;
}

std::vector<STProposal> STProposalBuilder::GenerateSTProposals(
    const STTopology& st_topology, const CorridorInfo::Type& type,
    const bool is_near_intersection) {
  const int n_layers = st_topology.size();
  const int n_proposals = n_layers + 1;
  std::vector<STProposal> st_proposals(n_proposals);
  for (int i = 0; i < n_proposals; ++i) {
    for (int j = 0; j < n_layers; ++j) {
      const bool is_front = (j < i);
      for (const auto& st_boudary : st_topology[j]) {
        bool is_filtered = false;
        const bool is_far_away_start_blocking_obs =
            st_boudary->min_t() > config_.t_start_blocking_max;
        const bool is_obs_behind_ego =
            !is_near_intersection && !is_front && st_boudary->min_s() < 0.0;
        if (type == CorridorInfo::Type::LANE_KEEP &&
            (is_far_away_start_blocking_obs || is_obs_behind_ego)) {
          is_filtered = true;
        }
        st_proposals[i].emplace_back(
            std::make_tuple(st_boudary, is_front, is_filtered));
      }
    }
  }
  return st_proposals;
}

void STProposalBuilder::SortAndFilterSTProposals(
    const STTopology& st_topology, const double v_extrap,
    std::unordered_map<const STBoundary*, double>& cost_map_front,
    std::unordered_map<const STBoundary*, double>& cost_map_rear,
    std::vector<std::pair<STProposal, double>>& cost_proposals,
    std::vector<STProposal>& st_proposals) {
  std::unordered_map<const STBoundary*, double> accel_map_front;
  std::unordered_map<const STBoundary*, double> accel_map_rear;
  ComputeAccelMap(st_topology, v_extrap, accel_map_front, accel_map_rear);

  auto AssignAccelCosts =
      [](const std::unordered_map<const STBoundary*, double>& accel_map,
         const double weight, const bool is_front) {
        std::unordered_map<const STBoundary*, double> cost_map;
        for (const auto& [st_boundary, accel] : accel_map) {
          cost_map[st_boundary] = weight * (is_front ? -std::min(accel, 0.0)
                                                     : std::max(accel, 0.0));
        }
        return cost_map;
      };

  cost_map_front = AssignAccelCosts(accel_map_front, config_.w_front, true);
  cost_map_rear = AssignAccelCosts(accel_map_rear, config_.w_rear, false);
  for (const auto& st_proposal : st_proposals) {
    double cost_accel = 0.0;
    double cost_narrow_gap = 0.0;
    double stboundary_count = 0.0;
    bool is_accel_too_large = false;
    for (const auto& [st_boundary, is_front, is_filtered] : st_proposal) {
      stboundary_count += 1.0;
      cost_accel += is_filtered ? 0.0
                                : (is_front ? cost_map_front.at(st_boundary)
                                            : cost_map_rear.at(st_boundary));

      is_accel_too_large =
          is_filtered
              ? false
              : (is_front
                     ? (accel_map_front.at(st_boundary) < config_.a_decel_limit)
                     : (accel_map_rear.at(st_boundary) >
                        a_acc_max_table_.Evaluate(v_extrap)));

      if (is_accel_too_large) {
        AINFO << "accel too large, break"
              << " accel map front = " << accel_map_front.at(st_boundary)
              << " accel map rear = " << accel_map_rear.at(st_boundary);
        break;
      }
    }
    if (is_accel_too_large) {
      continue;
    }

    cost_narrow_gap =
        narrow_gap_cost_table_.Evaluate(ComputeMinumumGap(st_proposal));

    cost_proposals.emplace_back(std::make_pair(
        st_proposal, cost_accel / stboundary_count + cost_narrow_gap));
  }

  std::sort(cost_proposals.begin(), cost_proposals.end(),
            [](const std::pair<STProposal, double>& p_1,
               const std::pair<STProposal, double>& p_2) {
              return p_1.second < p_2.second;
            });

  st_proposals.clear();
  for (size_t i = 0;
       i < std::min(static_cast<size_t>(config_.max_num_proposals),
                    cost_proposals.size());
       ++i) {
    const STProposal& proposal = cost_proposals[i].first;
    const double cost = cost_proposals[i].second;
    if (cost > config_.cost_max) {
      break;
    }
    st_proposals.emplace_back(proposal);
  }

  // if no valid proposals, generate an all-yield proposal
  if (st_proposals.empty()) {
    STProposal st_proposal_all_yield;
    for (int i = 0; i < st_topology.size(); ++i) {
      for (const auto& st_boudary : st_topology[i]) {
        st_proposal_all_yield.emplace_back(
            std::make_tuple(st_boudary, true, false));
      }
    }
    st_proposals.emplace_back(st_proposal_all_yield);
  }
}

void STProposalBuilder::ComputeAccelMap(
    const STTopology& st_topology, const double v_extrap,
    std::unordered_map<const STBoundary*, double>& accel_map_front,
    std::unordered_map<const STBoundary*, double>& accel_map_rear) {
  double s_ego = 0.0;
  for (const auto& layer_curr : st_topology) {
    for (const auto& st_boundary : layer_curr) {
      const double delta_t = st_boundary->lower_points().front().t();
      // ST boundary as a front
      const double soft_padding_front = 0.0;   // TODO: fill it in
      const double stiff_padding_front = 0.0;  // TODO: fill it in
      const double s_max = st_boundary->lower_points().front().s() -
                           stiff_padding_front - soft_padding_front;
      accel_map_front[st_boundary] =
          std::min(0.0, ComputeAccel(s_ego, v_extrap, s_max, delta_t));
      // ST boundary as a rear
      const double soft_padding_rear = 0.0;   // TODO: fill it in
      const double stiff_padding_rear = 0.0;  // TODO: fill it in
      const double s_min = st_boundary->upper_points().front().s() +
                           stiff_padding_rear + soft_padding_rear;
      accel_map_rear[st_boundary] =
          std::max(0.0, ComputeAccel(s_ego, v_extrap, s_min, delta_t));
    }
  }
}

double STProposalBuilder::ComputeAccel(double s_0, double v_0, double s_target,
                                       double delta_t) {
  static constexpr double kDeltaSMin = 1.0e-2;
  static constexpr double kTMin = 1.0e-2;
  double a = 0.0;

  if (s_target < s_0) {
    a = -std::numeric_limits<double>::infinity();
    return a;
  }
  if (delta_t <= kTMin) {
    a = std::numeric_limits<double>::infinity();
    return a;
  }
  a = 2.0 * (s_target - s_0 - v_0 * delta_t) / (delta_t * delta_t);
  if (v_0 + a * delta_t < 0.0) {
    const double delta_s = std::max(s_target - s_0, kDeltaSMin);
    a = std::max(-std::pow(v_0, 2) / (2.0 * delta_s), config_.a_min_stiff);
  }
  return a;
}

double STProposalBuilder::ComputeMinumumGap(const STProposal& st_proposal) {
  double gap_min = std::numeric_limits<double>::infinity();  // [m]

  if (st_proposal.size() < kSTProposalSizeMin) {
    return gap_min;
  }

  const double is_all_rear = !std::get<1>(st_proposal.front());
  const double is_all_front = std::get<1>(st_proposal.back());
  if (is_all_rear || is_all_front) {
    return gap_min;
  }

  std::vector<const STBoundary*> stboundary_front;
  std::vector<const STBoundary*> stboundary_rear;
  stboundary_front.clear();
  stboundary_rear.clear();

  for (const auto& [stboundary, is_front, is_filtered] : st_proposal) {
    if (!is_filtered) {
      if (is_front) {
        stboundary_front.emplace_back(stboundary);
      } else {
        stboundary_rear.emplace_back(stboundary);
      }
    }
  }

  double cur_min_time = 0.0;
  double cur_max_time = 0.0;

  double st_front_min_time = std::numeric_limits<double>::infinity();
  double st_front_max_time = 0.0;
  for (const STBoundary* st_front : stboundary_front) {
    cur_min_time = st_front->min_t();
    cur_max_time = st_front->max_t();

    if (cur_min_time < st_front_min_time) {
      st_front_min_time = cur_min_time;
    }

    if (cur_max_time > st_front_max_time) {
      st_front_max_time = cur_max_time;
    }
  }

  double st_rear_min_time = std::numeric_limits<double>::infinity();
  double st_rear_max_time = 0.0;
  for (const STBoundary* st_rear : stboundary_rear) {
    cur_min_time = st_rear->min_t();
    cur_max_time = st_rear->max_t();

    if (cur_min_time < st_rear_min_time) {
      st_rear_min_time = cur_min_time;
    }

    if (cur_max_time > st_rear_max_time) {
      st_rear_max_time = cur_max_time;
    }
  }

  double start_time = std::max(st_front_min_time, st_rear_min_time);
  double end_time = std::min(st_front_max_time, st_rear_max_time);
  if (start_time > end_time) {
    return gap_min;
  }

  double delta_time = 0.2;
  double t = start_time;
  while (t <= end_time) {
    double cur_front_min_s = std::numeric_limits<double>::infinity();
    double cur_rear_max_s = -std::numeric_limits<double>::infinity();

    for (const STBoundary* st_front : stboundary_front) {
      double cur_front_s = st_front->GetLowerSByT(t);

      if (cur_front_s < cur_front_min_s) {
        cur_front_min_s = cur_front_s;
      }
    }

    for (const STBoundary* st_rear : stboundary_rear) {
      double cur_rear_s = st_rear->GetLowerSByT(t);

      if (cur_rear_s > cur_rear_max_s) {
        cur_rear_max_s = cur_rear_s;
      }
    }

    double gap = cur_front_min_s - cur_rear_max_s;
    if (gap < gap_min) {
      gap_min = gap;
    }

    t += delta_time;
  }

  return gap_min;
}

std::vector<CorridorInfo::STProposal> STProposalBuilder::RecordSTProposals(
    const std::unordered_map<const STBoundary*, double>& cost_map_front,
    const std::unordered_map<const STBoundary*, double>& cost_map_rear,
    std::vector<std::pair<STProposal, double>>& cost_proposals) {
  std::vector<CorridorInfo::STProposal> st_proposals_record;
  for (const auto& [st_proposal, cost_total] : cost_proposals) {
    CorridorInfo::STProposal st_proposal_record;
    st_proposal_record.cost_total = cost_total;
    for (const auto& [st_boundary, is_front, is_filtered] : st_proposal) {
      st_proposal_record.st_boundaries.emplace_back(
          CorridorInfo::STProposal::STBoundary(st_boundary->id(), is_front));

      CorridorInfo::STProposal::Cost cost_record;
      if (is_front) {
        cost_record.reason = st_boundary->id() + ", front";
        cost_record.cost = cost_map_front.at(st_boundary);
      } else {
        cost_record.reason = st_boundary->id() + ", rear";
        cost_record.cost = cost_map_rear.at(st_boundary);
      }
      st_proposal_record.costs.emplace_back(std::move(cost_record));
    }
    st_proposals_record.emplace_back(std::move(st_proposal_record));
  }
  return st_proposals_record;
}

}  // namespace planning
}  // namespace zark
