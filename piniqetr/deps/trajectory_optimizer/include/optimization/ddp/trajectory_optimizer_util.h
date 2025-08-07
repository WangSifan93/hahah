#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "object/spacetime_object_state.h"
#include "optimization/ddp/trajectory_optimizer_defs.h"
#include "optimizer.pb.h"
#include "plan/trajectory_point.h"

namespace e2e_noa {
namespace planning {
namespace optimizer {

std::optional<std::vector<TrajectoryPoint>>
AdaptTrajectoryToGivenPlanStartPoint(int plan_id, int trajectory_steps,
                                     const Mfob& problem,
                                     const DdpOptimizerParamsProto& params,
                                     double max_adaption_cost,
                                     const TrajectoryPoint& plan_start_point,
                                     std::vector<TrajectoryPoint> trajectory);

absl::Status ValidateTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points,
    const TrajectoryOptimizerValidationParamsProto&
        trajectory_optimizer_validation_params,
    bool is_uturn, const TrajectoryOptimizerDebugProto& optimizer_debug);

bool HasSameDecisionOverSpacetimeObject(
    const std::vector<TrajectoryPoint>& traj_1,
    const std::vector<TrajectoryPoint>& traj_2,
    const std::vector<SpacetimeObjectState>& space_time_object_states);

}  
}  
}  

#endif
