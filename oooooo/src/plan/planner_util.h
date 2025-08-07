#ifndef ONBOARD_PLANNER_PLANNER_UTIL_H_
#define ONBOARD_PLANNER_PLANNER_UTIL_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/time/time.h"
#include "perception.pb.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/trajectory_point.h"
#include "planner_params.pb.h"
#include "positioning.pb.h"
#include "selector_params.pb.h"
#include "trajectory.pb.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

double ComputeLongitudinalJerk(const TrajectoryPoint& traj_point);
double ComputeLateralAcceleration(const TrajectoryPoint& traj_point);
double ComputeLateralJerk(const TrajectoryPoint& traj_point);

bool IsVulnerableRoadUserCategory(ObjectType type);
bool IsStaticObjectType(ObjectType type);

std::vector<ApolloTrajectoryPointProto> CreatePastPointsList(
    absl::Time plan_time, const TrajectoryProto& prev_traj, bool reset,
    int max_past_point_num);

ApolloTrajectoryPointProto ComputePlanStartPointAfterReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, double front_wheel_angle,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool is_forward_task);

ApolloTrajectoryPointProto ComputePlanStartPointAfterLateralReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, double front_wheel_angle,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

ApolloTrajectoryPointProto
ComputePlanStartPointAfterLongitudinalResetFromPrevTrajectory(
    const TrajectoryProto& prev_traj, const PoseProto& pose,
    double front_wheel_angle,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

SelectorParamsProto LoadSelectorConfigurationFromFile(
    const std::string& file_address);

}  // namespace planning
}  // namespace e2e_noa

#endif
