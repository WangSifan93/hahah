#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_BUILDER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_BUILDER_H_

#include <vector>

#include "absl/types/span.h"
#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_planner_object_trajectories.h"
#include "plan/planner_semantic_map_manager.h"
#include "planner_object.pb.h"
#include "planner_params.pb.h"
#include "router/plan_passage.h"
#include "trajectory_point.pb.h"
#include "util/hmi_content_util.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

struct SpacetimePlannerObjectTrajectoriesBuilderInput {
  const PlannerSemanticMapManager* psmm;
  const PlanPassage* passage;
  const PathSlBoundary* sl_boundary;
  const LaneChangeStateProto* lane_change_state;
  const VehicleGeometryParamsProto* veh_geom;
  const ApolloTrajectoryPointProto* plan_start_point;
  double st_planner_start_offset;
  const SpacetimePlannerObjectTrajectoriesProto* prev_st_trajs;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj;
  absl::Span<const ConstraintProto::StopLineProto> stop_lines;
  const SpacetimePlannerObjectTrajectoriesParamsProto*
      spacetime_planner_object_trajectories_params;
  const NudgeObjectInfo* nudge_object_info = nullptr;
};

SpacetimePlannerObjectTrajectories BuildSpacetimePlannerObjectTrajectories(
    const SpacetimePlannerObjectTrajectoriesBuilderInput& input,
    absl::Span<const SpacetimeObjectTrajectory> trajectories);

}  
}  

#endif
