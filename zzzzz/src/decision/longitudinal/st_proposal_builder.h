/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file: st_proposal_builder.h
 **/

#pragma once

#include <unordered_map>

#include "apps/planning/src/common/speed/st_boundary.h"
#include "apps/planning/src/decision/longitudinal/data_type.h"
#include "apps/planning/src/planning_msgs/task_config.h"
#include "apps/planning/src/common/corridor_info.h"
#include "apps/planning/src/common/lookup_table.h"

namespace zark {
namespace planning {

const int kSTProposalSizeMin = 2;

/**
 * class STProposalBuilder
 * @brief Build ST proposals from an ST topology.
 */
class STProposalBuilder {
 public:
  STProposalBuilder(const LongitudinalDeciderConfig::STProposalConfig& config);

  /**
   * @brief Build a set of ST proposals with an ST topology and an extrapolation
   * speed.
   *
   * @param st_topology an ST topology
   * @param v_extrap extrapolation speed [m/s]
   * @param type corridor type
   * @param is_near_intersection whether the ego is near an intersection
   * @return std::vector<STProposal> a set of ST proposals
   */
  std::vector<STProposal> BuildSTProposals(
      const STTopology& st_topology, const double v_extrap,
      const CorridorInfo::Type& type, const bool is_near_intersection,
      std::vector<CorridorInfo::STProposal>& st_proposals_record);

 private:
  /**
   * @brief Build a set of ST proposals with an ST topology.
   *
   * @param st_topology ST topology
   * @param type corridor type
   * @param is_near_intersection whether the ego is near an intersection
   * @return std::vector<STProposal> a set of ST proposals
   */
  std::vector<STProposal> GenerateSTProposals(
      const STTopology& st_topology, const CorridorInfo::Type& type,
      const bool is_near_intersection = false);

  /**
   * @brief Sort ST proposals by cost and filter.
   *
   * @param st_topology ST topology
   * @param v_extrap extrapolation speed [m/s]
   * @param cost_map_front the cost when treating the ST boundary as a front obs
   * @param cost_map_rear the cost when treating the ST boundary as a rear obs
   * @param cost_proposals <STProposal, cost> pairs
   * @param st_proposals ST proposals
   */
  void SortAndFilterSTProposals(
      const STTopology& st_topology, const double v_extrap,
      std::unordered_map<const STBoundary*, double>& cost_map_front,
      std::unordered_map<const STBoundary*, double>& cost_map_rear,
      std::vector<std::pair<STProposal, double>>& cost_proposals,
      std::vector<STProposal>& st_proposals);

  /**
   * @brief Compute cost of each ST boundary.
   *
   * @param st_topology ST topology
   * @param v_extrap extrapolation speed [m/s]
   * @param cost_map_front the cost when treating the ST boundary as a front obs
   * @param cost_map_rear the cost when treating the ST boundary as a rear obs
   */
  void ComputeAccelMap(
      const STTopology& st_topology, const double v_extrap,
      std::unordered_map<const STBoundary*, double>& cost_map_front,
      std::unordered_map<const STBoundary*, double>& cost_map_rear);

  /**
   * @brief Compute the acceleration needed to reach s_target within delta_t
   * given an initial state [s_0; v_0].
   *
   * @param s_0 initial position [m]
   * @param v_0 initial speed [m/s]
   * @param s_target target position [m]
   * @param delta_t time difference from s_0 to s_target [s]
   * @return double the uniform acceleration needed [m/s^2]
   */
  double ComputeAccel(const double s_0, const double v_0, const double s_target,
                      const double delta_t);

  /**
   * @brief Compute gap cost between all adjacent layer in current STProposal
   *
   * @param st_proposal
   * @return double gap cost
   */
  double ComputeMinumumGap(const STProposal& st_proposal);

  /**
   * @brief Convert ST proposals data to Corridor info ST proposals.
   * @param st_proposals ST proposals;
   * @return std::vector<CorridorInfo::STProposal> Corridor info ST proposals to
   * record
   */
  std::vector<CorridorInfo::STProposal> RecordSTProposals(
      const std::unordered_map<const STBoundary*, double>& cost_map_front,
      const std::unordered_map<const STBoundary*, double>& cost_map_rear,
      std::vector<std::pair<STProposal, double>>& cost_proposals);

 private:
  LongitudinalDeciderConfig::STProposalConfig config_;
  LookupTable a_acc_max_table_;
  LookupTable narrow_gap_cost_table_;

 private:
  FRIEND_TEST(STProposalBuilderTest, TestGenerateSTProposals);
  FRIEND_TEST(STProposalBuilderTest, TestSortAndFilterSTProposals);
  FRIEND_TEST(STProposalBuilderTest, TestComputeAccelMap);
  FRIEND_TEST(STProposalBuilderTest, TestComputeAccel);
  FRIEND_TEST(STProposalBuilderTest, TestComputeMinumumGap);
};

}  // namespace planning
}  // namespace zark
