/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file similarity_evaluator.cc
 **/

#include "apps/planning/src/evaluation/similarity_evaluator.h"
#include "apps/planning/src/evaluation/const_values.h"

namespace zark {
namespace planning {

using ::math::LineSegment2d;
using ::math::Vec2d;

SimilarityEvaluator::SimilarityEvaluator(
    EvaluationDeciderConfig::Similarity config)
    : config_(config),
      similarity_cost_table_(LookupTable(config.similarity_cost_table)) {}

Costs SimilarityEvaluator::Evaluate(const Proposal& proposal,
                                    const DiscretizedTrajectory& traj_prev) {
  constexpr int kMinNumPoints = 2;
  Costs costs;
  if (traj_prev.size() < kMinNumPoints) {
    return costs;
  }
  auto cur_trajectory = proposal.GetTrajectory();
  std::vector<double> l_diff;
  for (const auto& cur_point : cur_trajectory) {
    double min_distance_square = std::numeric_limits<double>::max();
    int min_index = 0;
    Vec2d xy_point(cur_point.path_point().x(), cur_point.path_point().y());
    for (std::size_t i = 0; i < traj_prev.size() - 1; ++i) {
      const LineSegment2d line_seg(Vec2d(traj_prev.at(i).path_point().x(),
                                         traj_prev.at(i).path_point().y()),
                                   Vec2d(traj_prev.at(i + 1).path_point().x(),
                                         traj_prev.at(i + 1).path_point().y()));

      const double distance_square = line_seg.DistanceSquareTo(xy_point);
      if (distance_square < min_distance_square) {
        min_distance_square = distance_square;
        min_index = i;
      }
    }
    const double min_distance = std::sqrt(min_distance_square);
    const LineSegment2d nearest_seg(
        Vec2d(traj_prev.at(min_index).path_point().x(),
              traj_prev.at(min_index).path_point().y()),
        Vec2d(traj_prev.at(min_index + 1).path_point().x(),
              traj_prev.at(min_index + 1).path_point().y()));
    const auto proj = nearest_seg.ProjectOntoUnit(xy_point);
    if (min_index == 0) {
      if (proj >= 0.0) {
        l_diff.emplace_back(min_distance);
      }
    } else if (min_index == static_cast<int>(traj_prev.size()) - 2) {
      if (proj <= 0.0) {
        l_diff.emplace_back(min_distance);
      } else {
        break;
      }
    } else {
      l_diff.emplace_back(min_distance);
    }
  }
  if (l_diff.empty()) {
    return costs;
  }
  const double l_average =
      std::accumulate(l_diff.begin(), l_diff.end(), 0.0) / l_diff.size();
  const double similarity_cost = similarity_cost_table_.Evaluate(l_average);
  if (similarity_cost > kCostEpsilon) {
    const std::string reason = "average difference to previous trajectory is " +
                               std::to_string(l_average) + "m. ";
    costs.emplace_back(std::make_pair(similarity_cost, reason));
  }
  return costs;
}

}  // namespace planning
}  // namespace zark
