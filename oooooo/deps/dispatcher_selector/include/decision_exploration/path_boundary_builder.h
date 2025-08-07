#ifndef PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_
#define PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_

#include "absl/status/statusor.h"
#include "common/path_sl_boundary.h"
#include "lane_change.pb.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "decision_exploration/smooth_reference_line_result.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromPlanPassage(
    const PlannerSemanticMapManager& psmm, const PlanPassage& plan_passage);

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromPose(
    const PlannerSemanticMapManager& psmm, const PlanPassage& plan_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const LaneChangeStateProto& lc_state,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    bool borrow_lane_boundary, bool should_smooth_next_left_turn);

}  

#endif
