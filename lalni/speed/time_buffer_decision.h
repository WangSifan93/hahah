#ifndef PLANNER_SPEED_TIME_BUFFER_DECIDER_H_
#define PLANNER_SPEED_TIME_BUFFER_DECIDER_H_

#include "object/spacetime_trajectory_manager.h"
#include "speed/st_boundary_with_decision.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

void DecideTimeBuffersForStBoundary(
    StBoundaryWithDecision* st_boundary_wd, double init_v,
    const VehicleGeometryParamsProto& vehicle_params,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    bool disable_pass_time_buffer);

}
}  // namespace e2e_noa

#endif
