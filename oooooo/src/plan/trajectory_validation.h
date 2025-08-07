#ifndef ONBOARD_PLANNER_TRAJECTORY_VALIDATION_H_
#define ONBOARD_PLANNER_TRAJECTORY_VALIDATION_H_

#include <string>
#include <vector>

#include "absl/types/span.h"
#include "async/thread_pool.h"
#include "common/path_sl_boundary.h"
#include "decision_exploration/decision_exploration_output.h"
#include "object/partial_spacetime_object_trajectory.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/trajectory_point.h"
#include "planner_params.pb.h"
#include "router/plan_passage.h"
#include "trajectory_point.pb.h"
#include "trajectory_validation.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

struct StObjectCollisionInfo {
  std::string object_id = "";
  double probability = 0.0;
};

void GetStObjectCollisionInfo(
    const PartialSpacetimeObjectTrajectory& st_object_with_time_range,
    const std::vector<TrajectoryPoint>& traj_points,
    const std::vector<Polygon2d>& av_box, double av_length,
    bool skip_no_decision_object, StObjectCollisionInfo* info);

std::vector<Polygon2d> GetAvBoxFromTrajPoints(
    const std::vector<TrajectoryPoint>& traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    int obs_check_horizon);

bool CheckTrajectoryControlConstraints(
    absl::Span<const TrajectoryPoint> traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    bool check_intrinsic_jerk, bool check_curvature_rate,
    TrajectoryValidationResultProto* result);

bool ValidateTrajectoryInternalConsistency(
    absl::Span<const TrajectoryPoint> traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    TrajectoryValidationResultProto* result);

bool ValidateTrajectory(
    absl::Span<const TrajectoryPoint> traj_points,
    const TrajectoryValidationOptionsProto& trajectory_validation_options,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    TrajectoryValidationResultProto* result);

bool CheckStimatedTrajectoryCurbCollision(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const TrajectoryPoint> traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    TrajectoryValidationResultProto* result);

bool ValidateStTrajectoryObjectCollision(
    const std::vector<PartialSpacetimeObjectTrajectory>& considered_st_objects,
    const std::vector<TrajectoryPoint>& traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    int obs_check_horizon, TrajectoryValidationResultProto* result,
    WorkerThreadManager* thread_pool);

bool ValidateTrajectoryOverlap(
    absl::Span<const ApolloTrajectoryPointProto> traj_points,
    absl::Span<const ApolloTrajectoryPointProto> past_points,
    TrajectoryValidationResultProto* result);

void ValidateTrajectoryLateralComfort(
    const std::vector<TrajectoryPoint>& traj_points,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params);

bool CheckPlanPassageBoundaryViolations(
    const std::vector<TrajectoryPoint>& traj_points,
    const PlanPassage& plan_passage, const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    TrajectoryValidationResultProto* result);

bool ValidateStTrajectory(
    const PlannerSemanticMapManager& psmm,
    const std::vector<PartialSpacetimeObjectTrajectory>& considered_st_objects,
    bool full_stop, const DecisionExplorationOutput& decision_exploration,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    TrajectoryValidationResultProto* result, WorkerThreadManager* thread_pool);

bool ValidateStPrevTrajectory(
    const PlannerSemanticMapManager& psmm,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    TrajectoryValidationResultProto* result);

bool ValidateFreespaceTrajectory(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    TrajectoryValidationResultProto* result);

bool ValidateAccTrajectory(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    absl::Span<const ApolloTrajectoryPointProto> past_traj,
    TrajectoryValidationResultProto* result);

}  // namespace planning
}  // namespace e2e_noa

#endif
