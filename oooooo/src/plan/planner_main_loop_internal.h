#ifndef ST_PLANNING_PLANNER_MAIN_LOOP_INTERNAL
#define ST_PLANNING_PLANNER_MAIN_LOOP_INTERNAL

#include <memory>
#include <tuple>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "alc.pb.h"
#include "autonomy_state.pb.h"
#include "chassis.pb.h"
#include "common/plan_start_point_info.h"
#include "lane_change.pb.h"
#include "maps/lane_path.h"
#include "math/vec.h"
#include "perception.pb.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/planner_state.h"
#include "planner.pb.h"
#include "planner_params.pb.h"
#include "positioning.pb.h"
#include "router/navi/route_navi_info.h"
#include "router/route_sections.h"
#include "router/route_sections_util.h"
#include "traffic_light_info.pb.h"
#include "trajectory.pb.h"
#include "trajectory_point.pb.h"
#include "trajectory_validation.pb.h"
#include "turn_signal.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

void FillTrajectoryProto(
    absl::Time plan_time,
    const std::vector<ApolloTrajectoryPointProto>& planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto>& past_points,
    const mapping::LanePath& target_lane_path_from_current,
    const LaneChangeStateProto& lane_change_state, TurnSignal turn_signal,

    const TrajectoryValidationResultProto& validate_result,
    TrajectoryProto* trajectory);

std::vector<ApolloTrajectoryPointProto> CreatePreviousTrajectory(
    absl::Time plan_time, const TrajectoryProto& previous_trajectory,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    bool reset);

PlanStartPointInfo ComputeStPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state,

    bool rerouted, bool aeb, double front_wheel_angle,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool override);

absl::Duration GetStPathPlanLookAheadDuration(
    const PlanStartPointInfo& plan_start_point_info, const PoseProto& pose,
    absl::Duration planned_look_ahead_time,
    const TrajectoryProto& previous_trajectory);

StPathPlanStartPointInfo GetStPathPlanStartPointInformation(
    const absl::Duration look_ahead_time,
    const PlanStartPointInfo& plan_start_point_info,
    const TrajectoryProto& previous_trajectory,
    std::optional<double> trajectory_optimizer_time_step,
    std::optional<absl::Time> last_st_path_plan_start_time);

absl::StatusOr<mapping::LanePath> FindPreferredLanePathFromTeleop(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections_from_start,
    const RouteNaviInfo& route_navi_info, mapping::ElementId ego_proj_lane_id,
    DriverAction::LaneChangeCommand lc_cmd);

absl::StatusOr<std::tuple<RouteSections, RouteSections, PointOnRouteSections>>
ProjectPointToRouteSections(const PlannerSemanticMapManager& psmm,
                            const RouteSections& route_sections,
                            const Vec2d& pos, double projection_range,
                            double keep_behind_length);

bool CheckIfAllowCancel(const ApolloTrajectoryPointProto& plan_start_point,
                        const VehicleGeometryParamsProto& vehicle_geometry,
                        const Vec2d& ego_pos,
                        e2e_noa::mapping::LanePath* preferred_lane_path,
                        const double& dist_buffer);

void HandleManualLcCommand(
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geometry,
    const PlannerSemanticMapManager& psmm,
    e2e_noa::DriverAction::LaneChangeCommand new_lc_cmd,
    const e2e_noa::mapping::LanePath& prev_lp_before_lc,
    const LaneChangeStateProto& prev_lc_state, const Vec2d& ego_pos,
    const double& ego_theta, e2e_noa::mapping::LanePath* preferred_lane_path,
    ALCState* alc_state, e2e_noa::DriverAction::LaneChangeCommand* lc_cmd_state,
    e2e_noa::planning::LcFeasibility* lc_unable_reason, bool* if_cancel_lc,
    int last_manual_lc_time, LaneChangeReason last_lc_reason);

}  // namespace planning
}  // namespace e2e_noa

#endif
