#ifndef PLANNER_SPEED_SPEED_FINDER_UTIL_H_
#define PLANNER_SPEED_SPEED_FINDER_UTIL_H_

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "absl/types/span.h"
#include "common/path_approx.h"
#include "common/vehicle_shape.h"
#include "math/segment_matcher/segment_matcher_kdtree.h"
#include "object/partial_spacetime_object_trajectory.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "object_scene_recognition.h"
#include "plan/discretized_path.h"
#include "speed/speed_bound.h"
#include "speed/speed_vector.h"
#include "speed/st_boundary.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/vt_speed_limit.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
#include "util/spacetime_util.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {
constexpr double kSpeedLimitProviderTimeStep = 0.1;

void KeepNearestStationarySpacetimeTrajectoryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision);

std::optional<std::string> GetNearestStationaryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision);

void ConfigureSpacetimeBoundaryDebug(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    SpeedPlanningDebugProto* speed_planning_proto);

std::vector<StBoundaryWithDecision> InitializeStBoundaryWithDecision(
    std::vector<StBoundaryRef> raw_st_boundaries);

int GetSpeedPlanningTrajectorySteps(double init_v, double default_speed_limit);

void PostProcessSpeedByFullStop(
    const SpeedPlanningParamsProto& speed_planning_params,
    SpeedVector* speed_data);

std::vector<Box2d> BuildAvHeadBoundingBoxes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const std::vector<VehicleShapeBasePtr>& av_shapes);

std::vector<VehicleShapeBasePtr> BuildAvShapes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const DiscretizedPath& path_points);

std::unique_ptr<KdtreeSegmentMatcher> BuildPathKdTree(
    const DiscretizedPath& path_points);

std::optional<PathApprox> BuildPathApproxForMirrors(
    const PathApprox& path_approx,
    const VehicleGeometryParamsProto& vehicle_geom);

std::vector<PartialSpacetimeObjectTrajectory> GetConsideredStObjects(
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& obj_mgr,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>
        processed_st_objects);

void CutoffSpeedByTimeHorizon(SpeedVector* speed_data);

SpeedVector GenerateReferenceSpeed(
    const std::vector<SpeedBoundWithInfo>& min_speed_limit, double init_v,
    double ref_speed_bias, double ref_speed_static_limit_bias, double max_accel,
    double max_decel, double total_time, double delta_t);

std::vector<std::pair<double, double>> GenerateAccelerationBound(
    const e2e_noa::SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const std::optional<double> acc_target_a,
    const std::optional<double> dec_target_a, double start_a, double delta_t,
    int knot_num);

bool IsConsiderOncomingObs(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const StBoundary& st_boundary, const PlanPassage plan_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params, double current_v,
    const DiscretizedPath& path);

}  // namespace planning
}  // namespace e2e_noa

#endif
