#ifndef PLANNER_SPEED_DECIDER_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_
#define PLANNER_SPEED_DECIDER_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_

#include <vector>

#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "plan/discretized_path.h"
#include "router/plan_passage.h"
#include "speed/st_graph_defs.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

std::vector<ConstraintProto::PathSpeedRegionProto>
MakeCloseObjectSlowdownDecision(
    const std::vector<CloseSpaceTimeObject>& close_space_time_objects,
    const PlanPassage& plan_passage, const DiscretizedPath& path_points,
    double av_speed, const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const bool is_on_highway,
    const bool is_near_consecutive_stationary_objects);

}
}  // namespace e2e_noa

#endif
