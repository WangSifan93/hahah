#ifndef ONBOARD_PLANNER_DECISION_PEDESTRIANS_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_PEDESTRIANS_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "maps/lane_path.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

absl::StatusOr<std::vector<ConstraintProto::SpeedRegionProto>>
BuildPedestriansConstraints(
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const PlanPassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const PathSlBoundary& sl_boundary,
    const SpacetimeTrajectoryManager& st_traj_mgr);

}
}  // namespace e2e_noa

#endif
