#ifndef PLANNER_SPEED_DECIDER_PRE_BRAKE_DECIDER_H_
#define PLANNER_SPEED_DECIDER_PRE_BRAKE_DECIDER_H_

#include <optional>
#include <vector>

#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "speed/speed_vector.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/vt_speed_limit.h"
#include "speed_planning_params.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

std::optional<VtSpeedLimit> MakePedestrainPreBrakeDecision(
    const SpeedPlanningParamsProto::PreBrakeDeciderParamsProto& params,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, double max_v, double time_step, int step_num,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

std::optional<VtSpeedLimit> MakeUncertainVehiclePreBrakeDecision(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_params, double current_v,
    double max_v, double time_step, int step_num,
    const SpeedVector& preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

std::optional<VtSpeedLimit> MakeCreepInteractionDecision(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_params, double current_v,
    double max_v, double time_step, int step_num,
    const SpeedVector& preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

}  // namespace planning
}  // namespace e2e_noa
#endif
