#ifndef PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
#define PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "async/thread_pool.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "planner_params.pb.h"
#include "speed/speed_limit_provider.h"
#include "speed/speed_vector.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_graph.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

absl::Status MakeSpeedSearchAndInteractiveDecision(
    std::string_view base_name, const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const DiscretizedPath& path, double current_v, double current_a,
    const SpeedPlanningParamsProto& speed_planning_params, double speed_cap,
    int traj_steps, SpeedLimitProvider* speed_limit_provider,
    SpeedVector* preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    InteractiveSpeedDebugProto* interactive_speed_debug,
    WorkerThreadManager* thread_pool,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const PlanPassage* plan_passage,
    const LaneChangeStateProto lane_change_state,
    const NudgeObjectInfo* nudge_object_info);
}
}  // namespace e2e_noa
#endif
