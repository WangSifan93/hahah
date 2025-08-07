#ifndef PLANNER_SPEED_INTERACTIVE_AGENT_PROCESS_H_
#define PLANNER_SPEED_INTERACTIVE_AGENT_PROCESS_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_graph.h"
#include "speed_planning.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

absl::Status InteractiveAgentProcess(
    const VehicleGeometryParamsProto& vehicle_geom, const StGraph& st_graph,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, double current_a,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const PlanPassage* plan_passage,
    const LaneChangeStateProto lane_change_state);

}

}  // namespace e2e_noa
#endif
