/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file: longitudinal_decider.h
 **/

#pragma once

#include <memory>
#include <string>

#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/decision/longitudinal/obs_sl_boundary_builder.h"
#include "apps/planning/src/decision/longitudinal/speed_limit_builder.h"
#include "apps/planning/src/decision/longitudinal/st_graph_builder.h"
#include "apps/planning/src/decision/longitudinal/st_proposal_builder.h"
#include "apps/planning/src/decision/longitudinal/st_topology_builder.h"
#include "apps/planning/src/tasks/task.h"

namespace zark {
namespace planning {
/**
 * class LongitudinalDecider
 * @brief longitudinal decider.
 */
class LongitudinalDecider : public Task {
 public:
  /**
   * @brief The constructor of LongitudinalDecider.
   * @param config config read from json file;
   * @param injector injector information;
   */
  LongitudinalDecider(const TaskConfig& config,
                      const std::shared_ptr<DependencyInjector>& injector);
  /**
   * @brief Execute function.
   * @param frame frame information;
   * @return Execute status
   */
  ::common::Status Execute(Frame* frame);

 private:
  /**
   * @brief Convert ST topology data to Corridor info ST topology.
   * @param st_topology ST topology data;
   * @return CorridorInfo::STTopology Corridor info ST topology to record
   */
  CorridorInfo::STTopology RecordSTTopology(const STTopology& st_topology);

  /**
   * @brief Init st boundary counter maps.
   */
  void InitSTBoundaryCounterMaps();

  /**
   * @brief Update st boundary counter maps according to corridorinfos.
   * @param corridor_infos A set of corridor info;
   */
  void UpdateSTBoundaryCounterMaps(
      const std::vector<CorridorInfo>& corridor_infos);

 private:
  std::unique_ptr<SpeedLimitBuilder> speed_limit_builder_;
  std::unique_ptr<ObsSLBoundaryBuilder> obs_sl_boundary_builder_;
  std::unique_ptr<STGraphBuilder> st_graph_builder_;
  std::unique_ptr<STTopologyBuilder> st_topology_builder_;
  std::unique_ptr<STProposalBuilder> st_proposal_builder_;

  std::unordered_map<CorridorInfo::Type, std::unordered_map<std::string, int>>
      st_boundary_counter_maps_;

 private:
  FRIEND_TEST(LongitudinalDeciderTest, TestRecordSTTopology);
  FRIEND_TEST(LongitudinalDeciderTest, TestInitSTBoundaryCounterMaps);
  FRIEND_TEST(LongitudinalDeciderTest, TestUpdateSTBoundaryCounterMaps);
};

}  // namespace planning
}  // namespace zark
