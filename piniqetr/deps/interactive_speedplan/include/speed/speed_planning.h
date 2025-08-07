#ifndef PLANNER_SPEED_SPEED_FINDER_H_
#define PLANNER_SPEED_SPEED_FINDER_H_

#include <string_view>

#include "absl/status/statusor.h"
#include "async/thread_pool.h"
#include "behavior.pb.h"
#include "planner_params.pb.h"
#include "speed/speed_planning_input.h"
#include "speed/speed_planning_output.h"
#include "speed_planning_params.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {
absl::StatusOr<SpeedPlanningOutput> ExecuteSpeedPlanning(
    const SpeedPlanningInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const SpeedPlanningParamsProto& speed_planning_params,
    WorkerThreadManager* thread_pool, EgoFrame* curr_ego_frame);
}
}  // namespace e2e_noa

#endif
