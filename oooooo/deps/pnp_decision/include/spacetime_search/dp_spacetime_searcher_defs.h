#ifndef SPACETIME_SEARCH_DP_MOTION_SEARCHER_DEFS_H_
#define SPACETIME_SEARCH_DP_MOTION_SEARCHER_DEFS_H_

#include <algorithm>
#include <array>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "math/util.h"
#include "plan/planner_defs.h"
#include "spacetime_search/spacetime_graph.h"
#include "trajectory_point.pb.h"
namespace e2e_noa::planning {

constexpr double kCanSetToZeroSpeed = 1.0;
constexpr double kSearchFailedCanSetToZeroSpeed = 1.5;
constexpr double kCanSetToZeroTrajLength = 4.0;
constexpr std::array<double, 9> kAccelerationSamplePoints = {
    -4.0, -3.0, -2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5};
constexpr double kDpDiscreteSpeedSampleStep = 3.0;
constexpr double kDpDiscreteTimeSampleStep = 2.5;
const int kDpDiscreteTimeHorizon = CeilToInteger(
    kInitializationTrajectoryTimeHorizon / kDpDiscreteTimeSampleStep);
constexpr double kMinSpeedForFinalCost = 3.0;
constexpr int kConstVelSampleLayerSizeThreshold = 5;

struct DpSpacetimeSample {
  int v_discrete;
  int t_discrete;
  DpSpacetimeSample(double v, double t)
      : v_discrete(std::max(0, RoundToInt(v / kDpDiscreteSpeedSampleStep))),
        t_discrete(std::clamp(RoundToInt(t / kDpDiscreteTimeSampleStep), 0,
                              kDpDiscreteTimeHorizon)) {}

  friend bool operator==(const DpSpacetimeSample& lhs,
                         const DpSpacetimeSample& rhs) {
    return lhs.v_discrete == rhs.v_discrete && lhs.t_discrete == rhs.t_discrete;
  }

  template <typename H>
  friend H AbslHashValue(H h, const DpSpacetimeSample& ms) {
    return H::combine(std::move(h), ms.v_discrete, ms.t_discrete);
  }
};

struct TrajInfo {
  SpacetimeEdgeIndex idx;
  double total_cost;
  std::vector<double> feature_costs;
  std::vector<ApolloTrajectoryPointProto> traj_points;
};

struct BestEdgeInfo {
  SpacetimeEdgeIndex idx;
  double total_cost = 0.0;
  bool is_created_stationary_spacetime = false;
};

enum class CollisionConfiguration {
  NONE = 0,
  FRONT = 1,

  LEFT = 2,

  RIGHT = 3,

  BACK = 4,

};
struct CollisionConfigurationInfo {
  int time_idx = 0;
  CollisionConfiguration collision_config = CollisionConfiguration::NONE;
};

using IgnoreTrajMap =
    absl::flat_hash_map<std::string, CollisionConfigurationInfo>;

}  

#endif
