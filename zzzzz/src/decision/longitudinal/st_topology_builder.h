/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file: st_topology_builder.h
 **/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "apps/planning/src/common/speed/st_boundary.h"
#include "apps/planning/src/decision/longitudinal/data_type.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

/**
 * class STTopologyBuilder
 * @brief build st topology with st graph and velocity extrapolation.
 */
class STTopologyBuilder {
 public:
  /**
   * @brief The constructor of STTopologyBuilder.
   * @param config config read from json file;
   */
  STTopologyBuilder(const LongitudinalDeciderConfig::STTopologyConfig& config);

  /**
   * @brief Build st topology with st graph and velocity extrapolation.
   *
   * @param st_graph st graph data
   * @param v_extrap velocity extrapolation
   * @return st topology
   */
  STTopology BuildSTTopology(const STGraph& st_graph, const double v_extrap);

 private:
  /**
   * @brief build dag with st graph and velocity extrapolation.
   *
   * @param st_graph st graph data
   * @param v_extrap velocity extrapolation
   * @return dag result
   */
  std::unordered_map<const STBoundary*, std::vector<const STBoundary*>>
  BuildDAG(const STGraph& st_graph, const double v_extrap);

  /**
   * @brief use dag result to sort and get st topology result.
   *
   * @param dag dag result
   * @param st_graph st graph data
   * @return st topology result
   */
  STTopology TopologicalSort(
      const std::unordered_map<const STBoundary*,
                               std::vector<const STBoundary*>>& dag);

  /**
   * @brief determine whether the edge from node_front to node_rear is valid.
   *
   * @param node_front st boundary node front
   * @param node_rear st boundary node rear
   * @param v_extrap velocity extrapolation
   * @return the result of edge validation
   */
  bool IsValidEdge(const STBoundary& node_front, const STBoundary& node_rear,
                   const double v_extrap);

  /**
   * @brief interpolation
   *
   * @param t_curr current time
   * @param upper_points upper points with time and s
   * @return the value interpolated by t_curr in upper_points
   */
  double Interpolation(double t_curr, const std::vector<STPoint>& upper_points);

 private:
  LongitudinalDeciderConfig::STTopologyConfig config_;
  FRIEND_TEST(STTopologyBuilderTest, TestBuildDAG);
  FRIEND_TEST(STTopologyBuilderTest, TestTopologicalSort);
  FRIEND_TEST(STTopologyBuilderTest, TestIsValidEdge);
  FRIEND_TEST(STTopologyBuilderTest, TestInterpolation);
};

}  // namespace planning
}  // namespace zark
