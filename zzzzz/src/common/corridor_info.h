/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file corridor_info.h
 **/

#pragma once

#include <unordered_map>
#include <vector>

#include "apps/planning/src/common/corridor.h"
#include "apps/planning/src/common/indexed_ptr_list.h"
#include "apps/planning/src/common/local_route/local_route.h"
#include "apps/planning/src/common/mission.h"
#include "apps/planning/src/common/speed/st_boundary.h"
#include "apps/planning/src/common/speed_limit.h"

namespace zark {
namespace planning {

constexpr int kCorridorTypeSize = 4;

class CorridorInfo {
 public:
  enum class Type { LANE_KEEP = 0, LANE_CHANGE = 1, STAGING = 2, NUDGE = 3 };
  static constexpr std::array<Type, kCorridorTypeSize> corridor_types = {
      Type::LANE_KEEP, Type::LANE_CHANGE, Type::STAGING, Type::NUDGE};

  struct STTopology {
    std::vector<std::vector<std::string>> layers;
  };

  struct STProposal {
    struct STBoundary {
      STBoundary() = default;
      STBoundary(const std::string obs_id, const bool is_front) {
        this->obs_id = obs_id;
        this->is_front = is_front;
      }
      std::string obs_id;
      bool is_front;
    };

    struct Cost {
      Cost() = default;
      Cost(const std::string& reason, const double cost) {
        this->reason = reason;
        this->cost = cost;
      }
      std::string reason;
      double cost;
    };

    std::vector<STBoundary> st_boundaries;
    std::vector<Cost> costs;
    double cost_total;
  };

  CorridorInfo() = delete;

  CorridorInfo(
      const LocalRoute& local_route, const Mission& mission,
      const CorridorInfo::Type& corridor_type, const Corridor& corridor,
      const int idx_start_point,
      const IndexedPtrList<std::string, const Obstacle*>& obstacle_map);

  CorridorInfo(const CorridorInfo& other) = default;
  CorridorInfo(CorridorInfo&& other) = default;

  CorridorInfo& operator=(const CorridorInfo& rhs) = delete;
  CorridorInfo& operator=(CorridorInfo&& rhs) = delete;

  inline const LocalRoute& GetLocalRoute() const { return local_route_; }

  inline const Mission& GetMission() const { return mission_; }

  inline const CorridorInfo::Type& GetType() const { return type_; }

  inline const Corridor& GetCorridor() const { return corridor_; }

  inline const int GetIdxStartPoint() const { return idx_start_point_; }

  inline const IndexedPtrList<std::string, const Obstacle*>& GetObstacleMap()
      const {
    return obstacle_map_;
  }

  inline const std::unordered_map<std::string, SpeedLimit>& GetSpeedLimitMap()
      const {
    return speed_limit_map_;
  }

  void SetSpeedLimitMap(
      const std::unordered_map<std::string, SpeedLimit>& speed_limit_map) {
    speed_limit_map_ = speed_limit_map;
  }

  inline const std::unordered_map<const Obstacle*, SLTrajectory>&
  GetObsSLBoundayMap() const {
    return obs_sl_boundary_map_;
  }

  void SetObsSLBoundayMap(
      const std::unordered_map<const Obstacle*, SLTrajectory>&
          obs_sl_boundary_map) {
    obs_sl_boundary_map_ = obs_sl_boundary_map;
  }

  inline const std::vector<STBoundary>& GetSTGraph() const { return st_graph_; }

  void SetSTGraph(const std::vector<STBoundary>& st_graph) {
    st_graph_ = st_graph;
  }

  inline const std::vector<std::pair<const Obstacle*, bool>>&
  GetLateralObstacles() const {
    return lateral_obstacles_;
  }

  void SetLateralObstacles(
      const std::vector<std::pair<const Obstacle*, bool>>& lateral_obstacles) {
    lateral_obstacles_ = lateral_obstacles;
  }

  inline const CorridorInfo::STTopology& GetSTTopology() const {
    return st_topology_;
  }

  void SetSTTopology(const CorridorInfo::STTopology& st_topology) {
    st_topology_ = st_topology;
  }

  inline const std::vector<CorridorInfo::STProposal>& GetSTProposals() const {
    return st_proposals_;
  }

  void SetSTProposals(
      const std::vector<CorridorInfo::STProposal>& st_proposals) {
    st_proposals_ = st_proposals;
  }

  inline const ::common::FrenetPoint& GetInitFrenetPoint() const {
    return init_frenet_point_;
  }

  void SetInitFrenetPoint(const ::common::FrenetPoint& init_frenet_point) {
    init_frenet_point_ = init_frenet_point;
  }

  inline const std::vector<Obstacle>& GetVirtualObstacles() const {
    return virtual_obstacles_;
  }

  inline std::vector<Obstacle>* MutableVirtualObstacles() {
    return &virtual_obstacles_;
  }

  inline IndexedPtrList<std::string, const Obstacle*>* MutableObstacleMap() {
    return &obstacle_map_;
  }

  inline const std::unordered_map<std::string, int>& GetSTBoundaryCounterMap()
      const {
    return st_boundary_counter_map_;
  }

  void SetSTBoundaryCounterMap(
      const std::unordered_map<std::string, int>& st_boundary_counter_map) {
    st_boundary_counter_map_ = st_boundary_counter_map;
  }

 private:
  const LocalRoute& local_route_;
  const Mission& mission_;
  const CorridorInfo::Type type_;
  const Corridor corridor_;
  const int idx_start_point_;  // index of planning start point in corridor
  IndexedPtrList<std::string, const Obstacle*> obstacle_map_;
  std::unordered_map<std::string, SpeedLimit> speed_limit_map_;
  std::unordered_map<const Obstacle*, SLTrajectory> obs_sl_boundary_map_;
  std::vector<STBoundary> st_graph_;
  std::vector<std::pair<const Obstacle*, bool>> lateral_obstacles_;
  CorridorInfo::STTopology st_topology_;
  std::vector<CorridorInfo::STProposal> st_proposals_;
  ::common::FrenetPoint init_frenet_point_;
  std::vector<Obstacle> virtual_obstacles_;
  std::unordered_map<std::string, int> st_boundary_counter_map_;
};

}  // namespace planning
}  // namespace zark
