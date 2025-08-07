#ifndef PLANNER_SPEED_DP_SVT_COST_H_
#define PLANNER_SPEED_DP_SVT_COST_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "math/piecewise_linear_function.h"
#include "object/spacetime_trajectory_manager.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/svt_graph_point.h"
#include "speed_planning_params.pb.h"

namespace e2e_noa::planning {
class DpSvtCost {
 public:
  DpSvtCost(const SpeedPlanningParamsProto* speed_planning_params,
            double total_t, double total_s,
            const std::vector<StBoundaryWithDecision*>*
                sorted_st_boundaries_with_decision,
            const SpacetimeTrajectoryManager& st_traj_mgr);

  std::vector<SvtGraphPoint::StBoundaryDecision>
  GetStBoundaryDecisionsForInitPoint(const SvtGraphPoint& svt_graph_point);
  void GetStBoundaryCostAndDecisions(
      const SvtGraphPoint& prev_svt_graph_point,
      const SvtGraphPoint& svt_graph_point, double av_pseed,
      double* st_boundary_cost,
      std::vector<SvtGraphPoint::StBoundaryDecision>* st_boundary_decisions);

  double GetSpatialPotentialCost(double s) const;

  double GetSpeedLimitCost(double speed, double speed_limit) const;

  double GetReferenceSpeedCost(double speed, double cruise_speed) const;

  double GetAccelCost(double accel) const;

  void SetFollowDistanceRelSpeedPlf(PiecewiseLinearFunction<double> plf) {
    follow_distance_rel_speed_plf_ = std::move(plf);
  }

 private:
  using SamplingDpSpeedParamsProto =
      SpeedPlanningParamsProto::SamplingDpSpeedParamsProto;

  const SpeedPlanningParamsProto* speed_planning_params_;
  const SamplingDpSpeedParamsProto* params_;

  PiecewiseLinearFunction<double> follow_distance_rel_speed_plf_;

  const std::vector<StBoundaryWithDecision*>*
      sorted_st_boundaries_with_decision_;
  absl::flat_hash_map<std::string, const StBoundaryWithDecision*>
      original_st_boundary_wd_map_;

  double unit_t_ = 0.0;
  double total_s_ = 0.0;

  absl::flat_hash_map<std::string, int> boundary_map_;

  std::vector<std::vector<std::pair<double, double>>> boundary_cost_;

  absl::Mutex boundary_cost_mutex_;
  const SpacetimeTrajectoryManager& st_traj_mgr_;
};
}  // namespace e2e_noa::planning

#endif
