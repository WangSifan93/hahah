#ifndef PLANNER_SPEED_GRIDDED_SVT_GRAPH_H_
#define PLANNER_SPEED_GRIDDED_SVT_GRAPH_H_

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "async/thread_pool.h"
#include "boost/container_hash/extensions.hpp"
#include "gflags/gflags.h"
#include "object/spacetime_trajectory_manager.h"
#include "speed/dp_svt_cost.h"
#include "speed/speed_limit_provider.h"
#include "speed/speed_vector.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_graph_data.h"
#include "speed/svt_graph_point.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"

DECLARE_bool(enable_sampling_dp_reference_speed);

namespace e2e_noa::planning {

struct SvGridIndex {
  int grid_index_s_ = 0;
  int grid_index_v_ = 0;

  SvGridIndex(int grid_index_s, int grid_index_v)
      : grid_index_s_(grid_index_s), grid_index_v_(grid_index_v) {}

  bool operator==(const SvGridIndex& other) const {
    return grid_index_s_ == other.grid_index_s_ &&
           grid_index_v_ == other.grid_index_v_;
  }
  bool operator!=(const SvGridIndex& other) const { return !(*this == other); }
  bool operator<(const SvGridIndex& other) const {
    return grid_index_s_ < other.grid_index_s_ ||
           (grid_index_s_ == other.grid_index_s_ &&
            grid_index_v_ < other.grid_index_v_);
  }

  struct HashFunction {
    std::size_t operator()(const SvGridIndex& index) const {
      std::size_t seed = 0;
      boost::hash_combine(seed, index.grid_index_s_);
      boost::hash_combine(seed, index.grid_index_v_);
      return seed;
    }
  };

  std::string DebugString() const {
    return absl::StrCat("(", grid_index_s_, ", ", grid_index_v_, ")");
  }
};

using SvGridIndexSet =
    absl::flat_hash_set<SvGridIndex, SvGridIndex::HashFunction>;

struct PreliminarySpeedWithCost {
  PreliminarySpeedWithCost() = default;
  PreliminarySpeedWithCost(double c, SpeedVector sv)
      : cost(c), preliminary_speed(std::move(sv)) {}
  double cost = 0.0;
  SpeedVector preliminary_speed;
};

class GriddedSvtGraph {
 public:
  GriddedSvtGraph(const StGraphData* st_graph_data, double init_v,
                  double init_a, double max_acceleration,
                  double max_deceleration,
                  const SpeedPlanningParamsProto* speed_planning_params,
                  double speed_cap,
                  std::vector<StBoundaryWithDecision*> st_boundaries_wd,
                  const SpacetimeTrajectoryManager& st_traj_mgr);

  void SwapStBoundariesWithDecision(
      std::vector<StBoundaryWithDecision*>* st_boundaries_with_decision) {
    sorted_st_boundaries_with_decision_.swap(*st_boundaries_with_decision);
  }

  absl::Status FindOptimalPreliminarySpeed(
      SpeedVector* preliminary_speed, SamplingDpDebugProto* sampling_dp_debug,
      WorkerThreadManager* thread_pool);

  absl::Status FindOptimalPreliminarySpeedWithCost(
      PreliminarySpeedWithCost* preliminary_speed_with_cost,
      SamplingDpDebugProto* sampling_dp_debug,
      WorkerThreadManager* thread_pool);

  absl::Status GenerateSamplingDpSpeedProfileCandidateSet(
      std::vector<PreliminarySpeedWithCost>* candidate_speed_profiles,
      SamplingDpDebugProto* sampling_dp_debug,
      InteractiveSpeedDebugProto::CandidateSet* candidate_set_debug,
      WorkerThreadManager* thread_pool);

 private:
  absl::StatusOr<SvGridIndexSet> SearchAndReturnFinalLayerPoints(
      WorkerThreadManager* thread_pool);

  absl::Status InitLayers();

  std::vector<SvtGraphPointRef> ExpandByConstAccModel(
      int cur_layer_index, double cur_t, double next_t, int next_point_index_t,
      const SpeedLimitProvider& speed_limit_provider, double cruise_speed,
      double init_speed, SvtGraphPoint* cur_point);

  void ExpandToNextLayer(
      int cur_layer_index, double cur_t, double next_t, int next_point_index_t,
      const SpeedLimitProvider& speed_limit_provider, double cruise_speed,
      double init_speed, SvtGraphPoint* cur_point,
      std::vector<std::vector<std::vector<SvtGraphPointRef>>>*
          next_layer_candidate_points,
      SvGridIndexSet* index_range);

  absl::StatusOr<PreliminarySpeedWithCost>
  GetSpeedProfileAndCompleteStBoundariesWithDecision(
      const SvGridIndexSet& final_layer_index_range);

  std::vector<PreliminarySpeedWithCost> SampleSpeedProfilesFromSamplingDp(
      const SvGridIndexSet& final_layer_indices);

 private:
  using SamplingDpSpeedParamsProto =
      SpeedPlanningParamsProto::SamplingDpSpeedParamsProto;

  const StGraphData* st_graph_data_;

  std::vector<StBoundaryWithDecision*> sorted_st_boundaries_with_decision_;

  double init_v_;
  double init_a_;
  const SpeedPlanningParamsProto* speed_planning_params_;
  const SamplingDpSpeedParamsProto* dp_params_;

  DpSvtCost dp_svt_cost_;

  double total_duration_t_ = 0.0;
  double unit_t_ = 0.0;
  double unit_inv_t_ = 0.0;
  int dimension_t_ = 0;

  double total_length_s_ = 0.0;
  double unit_s_ = 0.0;
  double unit_inv_s_ = 0.0;
  int dimension_grid_s_ = 0;

  double total_length_v_ = 0.0;
  double unit_v_ = 0.0;
  double unit_inv_v_ = 0.0;
  int dimension_grid_v_ = 0;
  double max_acceleration_ = 0.0;
  double max_deceleration_ = 0.0;

  std::vector<std::vector<double>> acc_matrix_;

  std::vector<double> t_knots_;
  std::vector<double> v_knots_;
  std::vector<double> s_knots_;

  std::vector<std::vector<std::vector<SvtGraphPointRef>>> layers_;
  const SpacetimeTrajectoryManager& st_traj_mgr_;
};
}  // namespace e2e_noa::planning

#endif
