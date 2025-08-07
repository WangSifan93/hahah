/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file comfort_evaluator.cc
 **/

#include "apps/planning/src/evaluation/comfort_evaluator.h"
#include "apps/planning/src/evaluation/const_values.h"

namespace zark {
namespace planning {

ComfortEvaluator::ComfortEvaluator(EvaluationDeciderConfig::Comfort config)
    : config_(config), time_weight_table_(config_.time_weight_table) {}

Costs ComfortEvaluator::Evaluate(const Proposal& proposal) {
  Costs comfort_cost;
  const auto& lat_mpc = proposal.GetLatMPCData();
  const auto& lon_mpc = proposal.GetLonMPCData();
  const auto& w_lat = config_.weight_lat_slack;
  const auto& w_lon = config_.weight_lon_slack;
  if (static_cast<int>(w_lat.weight_x.size()) == lat_mpc.n_x &&
      static_cast<int>(w_lat.weight_u.size()) == lat_mpc.n_u &&
      static_cast<int>(w_lat.weight_u_dot.size()) == lat_mpc.n_u_dot &&
      static_cast<int>(w_lon.weight_x.size()) == lon_mpc.n_x &&
      static_cast<int>(w_lon.weight_u.size()) == lon_mpc.n_u &&
      static_cast<int>(w_lon.weight_u_dot.size()) == lon_mpc.n_u_dot) {
    SlackCost(lat_mpc.slacks.soft_min, w_lat, time_weight_table_, lat_mpc.names,
              comfort_cost);
    SlackCost(lat_mpc.slacks.soft_max, w_lat, time_weight_table_, lat_mpc.names,
              comfort_cost);
    SlackCost(lon_mpc.slacks.soft_min, w_lon, time_weight_table_, lon_mpc.names,
              comfort_cost);
    SlackCost(lon_mpc.slacks.soft_max, w_lon, time_weight_table_, lon_mpc.names,
              comfort_cost);
  }
  return comfort_cost;
}

void ComfortEvaluator::SlackSliceCost(const Eigen::MatrixXd& slack_slice,
                                      const std::vector<double>& weight,
                                      const LookupTable& t_weight_tbl,
                                      const std::vector<std::string>& name,
                                      Costs& costs) {
  // col size of slack_slice is 41, and total time is 4s
  for (int row = 0; row < slack_slice.rows(); ++row) {
    double cost{0.0};
    constexpr double interval_time = 0.2;
    for (int col = 0; col < slack_slice.cols(); ++col) {
      const double t_weight = t_weight_tbl.Evaluate(col * interval_time);
      const auto slice_cost = slack_slice(row, col) * weight.at(row) * t_weight;
      if (slice_cost > kSlackEpsilon) {
        cost = std::max(cost, slice_cost);
      }
    }  // col
    if (cost > kCostEpsilon) {
      const std::string reason = "soft constraint violation in " + name.at(row);
      costs.emplace_back(std::make_pair(cost, reason));
    }
  }  // row
}

void ComfortEvaluator::SlackCost(
    const MPCData::Slacks::Slack& slack,
    const EvaluationDeciderConfig::SlackWeight& s_weight,
    const LookupTable& t_weight_tbl, const MPCData::Names& names,
    Costs& costs) {
  SlackSliceCost(slack.x, s_weight.weight_x, t_weight_tbl, names.x, costs);
  SlackSliceCost(slack.u, s_weight.weight_u, t_weight_tbl, names.u, costs);
  SlackSliceCost(slack.u_dot, s_weight.weight_u_dot, t_weight_tbl, names.u_dot,
                 costs);
}

}  // namespace planning
}  // namespace zark
