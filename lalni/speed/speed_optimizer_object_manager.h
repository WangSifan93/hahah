#ifndef PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_MANAGER_H_
#define PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_MANAGER_H_

#include <vector>

#include "absl/types/span.h"
#include "lane_change.pb.h"
#include "object/spacetime_trajectory_manager.h"
#include "speed/speed_optimizer_object.h"
#include "speed/st_boundary_with_decision.h"
#include "speed_planning_params.pb.h"

namespace e2e_noa::planning {

enum SpeedOptimizerObjectType {
  MOVING_FOLLOW = 0,
  MOVING_LEAD = 1,
  STATIONARY = 2,
};

class SpeedOptimizerObjectManager {
 public:
  SpeedOptimizerObjectManager(
      absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
      const SpacetimeTrajectoryManager& traj_mgr, double av_speed,
      double plan_total_time, double plan_time_interval,
      const SpeedPlanningParamsProto& speed_planning_params,
      const LaneChangeStage& lc_stage);

  absl::Span<const SpeedOptimizerObject> MovingFollowObjects() const {
    return objects_.at(SpeedOptimizerObjectType::MOVING_FOLLOW);
  }

  absl::Span<const SpeedOptimizerObject> MovingLeadObjects() const {
    return objects_.at(SpeedOptimizerObjectType::MOVING_LEAD);
  }

  absl::Span<const SpeedOptimizerObject> StationaryObjects() const {
    return objects_.at(SpeedOptimizerObjectType::STATIONARY);
  }

  const std::string& attention_obj_id() const { return attention_obj_id_; }

 private:
  std::vector<std::vector<SpeedOptimizerObject>> objects_;
  std::string attention_obj_id_ = "";
};

}  // namespace e2e_noa::planning

#endif
