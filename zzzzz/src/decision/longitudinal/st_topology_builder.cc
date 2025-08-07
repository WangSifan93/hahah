/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file: st_topology_builder.cc
 **/

#include <algorithm>
#include <memory>
#include <queue>
#include "apps/planning/src/decision/longitudinal/st_topology_builder.h"

namespace zark {
namespace planning {

STTopologyBuilder::STTopologyBuilder(
    const LongitudinalDeciderConfig::STTopologyConfig& config) {
  config_ = config;
}

STTopology STTopologyBuilder::BuildSTTopology(const STGraph& st_graph,
                                              const double v_extrap) {
  const std::unordered_map<const STBoundary*, std::vector<const STBoundary*>>&
      dag = BuildDAG(st_graph, v_extrap);
  return TopologicalSort(dag);
}

std::unordered_map<const STBoundary*, std::vector<const STBoundary*>>
STTopologyBuilder::BuildDAG(const STGraph& st_graph, const double v_extrap) {
  std::unordered_map<const STBoundary*, std::vector<const STBoundary*>> dag;
  const int num_st_boundary = st_graph.size();
  for (int i = 0; i < num_st_boundary; ++i) {
    const STBoundary& node_1 = st_graph[i];
    if (!dag.count(&node_1)) {
      dag[&node_1].clear();
    }
    for (int j = 0; j <= i - 1; ++j) {
      const STBoundary& node_2 = st_graph[j];
      if (IsValidEdge(node_1, node_2, v_extrap)) {
        dag[&node_1].emplace_back(&node_2);
      }
      if (IsValidEdge(node_2, node_1, v_extrap)) {
        dag[&node_2].emplace_back(&node_1);
      }
    }
  }
  return dag;
}

STTopology STTopologyBuilder::TopologicalSort(
    const std::unordered_map<const STBoundary*, std::vector<const STBoundary*>>&
        dag) {
  int n_nodes = dag.size();
  std::unordered_map<const STBoundary*, int> in_deg;

  for (const auto& [parent, children] : dag) {
    if (!in_deg.count(parent)) {
      in_deg[parent] = 0;
    }
    for (auto child : children) {
      in_deg[child]++;
    }
  }

  std::queue<const STBoundary*> q;
  STTopology st_topology;
  std::vector<const STBoundary*> layer_curr;
  for (const auto& in : in_deg) {
    if (in.second == 0) {
      q.push(in.first);
      layer_curr.emplace_back(in.first);
    }
  }
  if (layer_curr.size() > 0) {
    st_topology.emplace_back(layer_curr);
  }
  int count = 0;
  while (!q.empty()) {
    int n_q = q.size();
    layer_curr.clear();
    for (int i = 0; i < n_q; ++i) {
      const STBoundary* node_curr = q.front();
      q.pop();
      count++;
      for (const auto& node_next : dag.at(node_curr)) {
        in_deg[node_next]--;
        if (in_deg[node_next] == 0) {
          q.push(node_next);
          layer_curr.emplace_back(node_next);
        }
      }
    }
    if (layer_curr.size() > 0) {
      st_topology.emplace_back(layer_curr);
    }
  }

  if (count < n_nodes) {
    st_topology.clear();
    layer_curr.clear();
    for (const auto& dag_item : dag) {
      layer_curr.emplace_back(dag_item.first);
    }
    if (layer_curr.size() > 0) {
      st_topology.emplace_back(layer_curr);
    }
  }
  return st_topology;
}

bool STTopologyBuilder::IsValidEdge(const STBoundary& node_front,
                                    const STBoundary& node_rear,
                                    const double v_extrap) {
  bool is_valid = true;
  if (node_rear.upper_points().size() < 1) {
    return false;
  }
  for (const auto& lower_point : node_front.lower_points()) {
    const double t_curr = lower_point.t();
    const double s_curr = lower_point.s();
    if (t_curr < node_rear.min_t()) {
      if (s_curr < node_rear.upper_points().front().s() +
                       v_extrap * (t_curr - node_rear.min_t())) {
        is_valid = false;
      }
    } else if (t_curr >= node_rear.min_t() && t_curr <= node_rear.max_t()) {
      if (s_curr < Interpolation(t_curr, node_rear.upper_points())) {
        is_valid = false;
      }
    } else {
      if (s_curr <
          node_rear.upper_points().back().s() +
              config_.alpha * v_extrap * (t_curr - node_rear.max_t())) {
        is_valid = false;
      }
    }
  }
  return is_valid;
}

double STTopologyBuilder::Interpolation(
    double t_curr, const std::vector<STPoint>& upper_points) {
  int input_size = upper_points.size();
  if (input_size < 1) {
    return 0.0;
  } else if (input_size < 2) {
    return upper_points.front().s();
  }

  size_t match_idx = 0;
  if (t_curr < upper_points.front().t()) {
    return upper_points.front().s();
  } else if (t_curr >= upper_points.back().t()) {
    return upper_points.back().s();
  } else {
    for (int j = 1; j < input_size; j++) {
      if (t_curr >= upper_points[j - 1].t() && t_curr < upper_points[j].t()) {
        match_idx = j - 1;
        break;
      }
    }
    return upper_points[match_idx].s() +
           (t_curr - upper_points[match_idx].t()) *
               (upper_points[match_idx + 1].s() - upper_points[match_idx].s()) /
               (upper_points[match_idx + 1].t() - upper_points[match_idx].t());
  }
}

}  // namespace planning
}  // namespace zark
